/*
 * TUM Autonomous Motorsport Georeferencing Tool
 * Copyright (C) 2026 Maximilian Leitenstern
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#include "rosbag_io.hpp"

#include <tf2/exceptions.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <vector>
#include <string>
#include <utility>
namespace flexcloud
{
constexpr const char * NAVSATFIX_MSG = "sensor_msgs/msg/NavSatFix";
constexpr const char * ODOMETRY_MSG = "nav_msgs/msg/Odometry";
std::int64_t to_nanosec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
tf2::TimePoint to_tf2_time(const std::int64_t stamp)
{
  return tf2::TimePoint(std::chrono::nanoseconds(stamp));
}
rosbag_io::rosbag_io(
  const std::string & bag_path, const std::string & topic, const std::string & target_frame,
  float stddev_threshold, const std::optional<Eigen::Vector3d> & origin)
: bag_path_(bag_path),
  topic_(topic),
  target_frame_(target_frame),
  stddev_threshold_(stddev_threshold),
  origin_(origin),
  reader_(bag_path),
  tf_buffer_(std::chrono::hours(24)),
  logger_(rclcpp::get_logger("flexcloud_rosbag_io"))
{
  // Resolve message type from metadata of the already-opened reader.
  topic_type_ = resolve_topic_type();
  if (topic_type_.empty()) {
    throw std::runtime_error("Topic '" + topic_ + "' not found in bag '" + bag_path_ + "'");
  }
  if (topic_type_ != NAVSATFIX_MSG && topic_type_ != ODOMETRY_MSG) {
    throw std::runtime_error(
      "Unsupported topic type '" + topic_type_ +
      "' (only sensor_msgs/msg/NavSatFix and nav_msgs/msg/Odometry are supported)");
  }
  RCLCPP_INFO(logger_, "Reading topic '%s' (%s) from bag", topic_.c_str(), topic_type_.c_str());

  // TF listeners — same callback handles both /tf and /tf_static; the
  // is_static flag is taken from bag_msg.topic_name.
  if (!target_frame_.empty()) {
    reader_.add_listener<tf2_msgs::msg::TFMessage>(
      "/tf", std::bind(&rosbag_io::tf_callback, this, std::placeholders::_1));
    reader_.add_listener<tf2_msgs::msg::TFMessage>(
      "/tf_static", std::bind(&rosbag_io::tf_callback, this, std::placeholders::_1));
  }

  // Topic listener and projection setup.
  if (topic_type_ == NAVSATFIX_MSG) {
    if (!origin_.has_value()) {
      throw std::runtime_error("No origin provided for NavSatFix projection!");
    }
    proj_.set_origin(*origin_);
    RCLCPP_INFO(
      logger_, "NavSatFix origin (custom): %f %f %f (geoid height %.3f m)", origin_->x(),
      origin_->y(), origin_->z(), proj_.geoid_height());
    reader_.add_listener<sensor_msgs::msg::NavSatFix>(
      topic_, std::bind(&rosbag_io::navsatfix_callback, this, std::placeholders::_1));
  } else {
    if (origin_.has_value()) {
      RCLCPP_WARN(
        logger_, "Origin provided but will be ignored for Odometry topic '%s'", topic_.c_str());
    }
    reader_.add_listener<nav_msgs::msg::Odometry>(
      topic_, std::bind(&rosbag_io::odometry_callback, this, std::placeholders::_1));
  }
}
std::string rosbag_io::resolve_topic_type()
{
  for (const auto & info : reader_.get_metadata().topics_with_message_count) {
    if (info.topic_metadata.name == topic_) {
      return info.topic_metadata.type;
    }
  }
  return "";
}
void rosbag_io::tf_callback(const tools::RosbagReaderMsg<tf2_msgs::msg::TFMessage> & msg)
{
  const bool is_static = msg.bag_msg.topic_name == "/tf_static";
  for (const auto & t : msg.ros_msg.transforms) {
    if (t.header.frame_id == t.child_frame_id) continue;
    tf_buffer_.setTransform(t, "bag", is_static);
    tf_parents_[t.child_frame_id] = t.header.frame_id;
    if (is_static) {
      ++n_static_tf_;
    } else {
      ++n_dynamic_tf_;
    }
  }
}
void rosbag_io::navsatfix_callback(const tools::RosbagReaderMsg<sensor_msgs::msg::NavSatFix> & msg)
{
  const auto & fix = msg.ros_msg;
  if (source_frame_.empty()) {
    source_frame_ = fix.header.frame_id;  // the antenna; the world frame is inferred from TF
  }
  const double x_stddev = std::sqrt(std::max(0.0, fix.position_covariance[0]));
  const double y_stddev = std::sqrt(std::max(0.0, fix.position_covariance[4]));
  const double z_stddev = std::sqrt(std::max(0.0, fix.position_covariance[8]));
  if (std::sqrt(x_stddev * x_stddev + y_stddev * y_stddev) > stddev_threshold_) {
    ++skipped_stddev_;
    return;
  }

  // Project lat/lon/alt → ENU using the shared ENUProjection helper.
  const Eigen::Vector3d p = proj_.forward(fix.latitude, fix.longitude, fix.altitude);

  positions_.emplace_back(
    PointStdDev(p.x(), p.y(), p.z(), x_stddev, y_stddev, z_stddev), to_nanosec(fix.header.stamp));
}
void rosbag_io::odometry_callback(const tools::RosbagReaderMsg<nav_msgs::msg::Odometry> & msg)
{
  const auto & odom = msg.ros_msg;
  if (source_frame_.empty()) {
    source_frame_ = odom.child_frame_id;
    world_frame_ = odom.header.frame_id;
  }
  const double x_stddev = std::sqrt(std::max(0.0, odom.pose.covariance[0]));
  const double y_stddev = std::sqrt(std::max(0.0, odom.pose.covariance[7]));
  const double z_stddev = std::sqrt(std::max(0.0, odom.pose.covariance[14]));
  if (std::sqrt(x_stddev * x_stddev + y_stddev * y_stddev) > stddev_threshold_) {
    ++skipped_stddev_;
    return;
  }
  const auto & p = odom.pose.pose.position;
  positions_.emplace_back(
    PointStdDev(p.x, p.y, p.z, x_stddev, y_stddev, z_stddev), to_nanosec(odom.header.stamp));
}
std::string rosbag_io::tf_root(const std::string & frame) const
{
  std::string root = frame;
  for (std::size_t i = 0; i < tf_parents_.size(); ++i) {  // bounded in case of a cycle
    const auto parent = tf_parents_.find(root);
    if (parent == tf_parents_.end()) {
      break;
    }
    root = parent->second;
  }
  if (root == frame) {
    throw std::runtime_error("Frame '" + frame + "' has no parent in /tf");
  }
  return root;
}
void rosbag_io::resolve_positions()
{
  if (target_frame_.empty() || positions_.empty() || source_frame_ == target_frame_) {
    return;
  }
  if (source_frame_.empty()) {
    throw std::runtime_error("Messages on '" + topic_ + "' have no frame id to apply TF to");
  }
  // NavSatFix names no world frame: use the TF root, which has to be ENU-aligned like the
  // projection (only orientation is used)
  const std::string world = world_frame_.empty() ? tf_root(source_frame_) : world_frame_;
  if (world_frame_.empty()) {
    RCLCPP_INFO(
      logger_, "Lever arm rotated with the orientation of TF root '%s' (must be ENU-aligned)",
      world.c_str());
  }
  // Origin of `frame` expressed in the world frame
  const auto origin = [this, &world](const std::string & frame, const tf2::TimePoint & stamp) {
    const auto t = tf_buffer_.lookupTransform(world, frame, stamp).transform.translation;
    return Eigen::Vector3d(t.x, t.y, t.z);
  };

  std::string first_error;
  std::vector<PointStdDevStamped> moved;
  moved.reserve(positions_.size());
  for (auto pos : positions_) {
    try {
      // Lever arm from the source frame to target_frame in world axes: the orientation and the
      // offset come from TF, the absolute position from the message
      const tf2::TimePoint stamp = to_tf2_time(pos.stamp);
      pos.point.pos += origin(target_frame_, stamp) - origin(source_frame_, stamp);
      moved.push_back(pos);
    } catch (const tf2::TransformException & ex) {
      if (skipped_tf_++ == 0) {
        first_error = ex.what();
      }
    }
  }
  if (skipped_tf_ > 0) {
    RCLCPP_WARN(
      logger_, "Dropped %zu of %zu messages without a transform '%s' → '%s' (first error: %s)",
      skipped_tf_, positions_.size(), source_frame_.c_str(), target_frame_.c_str(),
      first_error.c_str());
  }
  positions_ = std::move(moved);
}
std::vector<PointStdDevStamped> rosbag_io::run()
{
  // Process the bag
  reader_.process();

  if (!target_frame_.empty()) {
    RCLCPP_INFO(
      logger_, "Loaded %zu static and %zu dynamic transforms from bag", n_static_tf_,
      n_dynamic_tf_);
  }
  if (skipped_stddev_ > 0) {
    RCLCPP_INFO(logger_, "Skipped %zu reference frames due to stddev threshold", skipped_stddev_);
  }

  // TF is resolved only now, with every /tf and /tf_static message in the buffer
  resolve_positions();

  std::sort(
    positions_.begin(), positions_.end(),
    [](const PointStdDevStamped & a, const PointStdDevStamped & b) { return a.stamp < b.stamp; });

  RCLCPP_INFO(logger_, "Loaded %zu reference positions from bag", positions_.size());
  return std::move(positions_);
}
}  // namespace flexcloud
