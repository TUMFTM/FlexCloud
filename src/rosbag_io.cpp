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

#include <GeographicLib/NormalGravity.hpp>
#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>
#include <vector>
#include <tf2_eigen/tf2_eigen.hpp>
#include <utility>

namespace flexcloud
{
namespace
{
constexpr const char * NAVSATFIX_MSG = "sensor_msgs/msg/NavSatFix";
constexpr const char * ODOMETRY_MSG = "nav_msgs/msg/Odometry";

std::int64_t to_nanosec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

tf2::TimePoint to_tf2_time(const builtin_interfaces::msg::Time & stamp)
{
  return tf2::TimePoint(std::chrono::nanoseconds(to_nanosec(stamp)));
}

GeographicLib::Geocentric make_wgs84_ellipsoid()
{
  const auto & earth = GeographicLib::NormalGravity::WGS84();
  return {earth.EquatorialRadius(), earth.Flattening()};
}
}  // namespace

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
  ellipsoid_(make_wgs84_ellipsoid()),
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
  RCLCPP_INFO(
    logger_, "Reading topic '%s' (%s) from bag", topic_.c_str(), topic_type_.c_str());

  // Custom origin (NavSatFix only — Odometry positions are already cartesian).
  if (origin_.has_value() && topic_type_ == NAVSATFIX_MSG) {
    proj_.emplace(origin_->x(), origin_->y(), origin_->z(), ellipsoid_);
    RCLCPP_INFO(
      logger_, "NavSatFix origin (custom): %f %f %f", origin_->x(), origin_->y(), origin_->z());
  }

  // Listeners — TF first, then the position topic. All on the same reader so
  // process() iterates the bag exactly once.
  if (!target_frame_.empty()) {
    reader_.add_listener<tf2_msgs::msg::TFMessage>(
      "/tf", std::bind(&rosbag_io::tf_callback, this, std::placeholders::_1));
    reader_.add_listener<tf2_msgs::msg::TFMessage>(
      "/tf_static", std::bind(&rosbag_io::tf_callback, this, std::placeholders::_1));
  }

  if (topic_type_ == NAVSATFIX_MSG) {
    reader_.add_listener<sensor_msgs::msg::NavSatFix>(
      topic_, std::bind(&rosbag_io::navsatfix_callback, this, std::placeholders::_1));
  } else {
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
    if (is_static) {
      ++n_static_tf_;
    } else {
      ++n_dynamic_tf_;
    }
  }
}

void rosbag_io::navsatfix_callback(
  const tools::RosbagReaderMsg<sensor_msgs::msg::NavSatFix> & msg)
{
  const auto & fix = msg.ros_msg;
  const double x_stddev = std::sqrt(std::max(0.0, fix.position_covariance[0]));
  const double y_stddev = std::sqrt(std::max(0.0, fix.position_covariance[4]));
  const double z_stddev = std::sqrt(std::max(0.0, fix.position_covariance[8]));
  if (std::sqrt(x_stddev * x_stddev + y_stddev * y_stddev) > stddev_threshold_) {
    ++skipped_stddev_;
    return;
  }
  raw_navsat_.push_back(
    {fix.latitude, fix.longitude, fix.altitude, x_stddev, y_stddev, z_stddev,
     fix.header.frame_id, fix.header.stamp});
}

void rosbag_io::odometry_callback(
  const tools::RosbagReaderMsg<nav_msgs::msg::Odometry> & msg)
{
  const auto & odom = msg.ros_msg;
  const double x_stddev = std::sqrt(std::max(0.0, odom.pose.covariance[0]));
  const double y_stddev = std::sqrt(std::max(0.0, odom.pose.covariance[7]));
  const double z_stddev = std::sqrt(std::max(0.0, odom.pose.covariance[14]));
  if (std::sqrt(x_stddev * x_stddev + y_stddev * y_stddev) > stddev_threshold_) {
    ++skipped_stddev_;
    return;
  }
  raw_odom_.push_back(
    {{odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z},
     x_stddev, y_stddev, z_stddev, odom.child_frame_id, odom.header.stamp});
}

Eigen::Vector3d rosbag_io::transform_offset(
  const std::string & src_frame, const tf2::TimePoint & stamp) const
{
  if (target_frame_.empty() || src_frame.empty() || target_frame_ == src_frame) {
    return Eigen::Vector3d::Zero();
  }
  try {
    // translation from the static (latest-available) transform
    auto t_static =
      tf_buffer_.lookupTransform(target_frame_, src_frame, tf2::TimePointZero).transform;
    // orientation from the dynamic transform at the message timestamp;
    // fall back to the static one if the requested time is outside the dynamic window
    geometry_msgs::msg::Transform t_dyn;
    try {
      t_dyn = tf_buffer_.lookupTransform(target_frame_, src_frame, stamp).transform;
    } catch (const tf2::TransformException &) {
      t_dyn = t_static;
    }
    Eigen::Quaterniond rot(
      t_dyn.rotation.w, t_dyn.rotation.x, t_dyn.rotation.y, t_dyn.rotation.z);
    Eigen::Vector3d trans_static(
      t_static.translation.x, t_static.translation.y, t_static.translation.z);
    return rot * trans_static;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      logger_, "No transform from '%s' to '%s': %s — using zero offset", src_frame.c_str(),
      target_frame_.c_str(), ex.what());
    return Eigen::Vector3d::Zero();
  }
}

std::vector<PointStdDevStamped> rosbag_io::run()
{
  // Single pass over the bag: TF and position callbacks fire as messages stream.
  reader_.process();

  if (!target_frame_.empty()) {
    RCLCPP_INFO(
      logger_, "Loaded %zu static and %zu dynamic transforms from bag", n_static_tf_,
      n_dynamic_tf_);
  }

  std::vector<PointStdDevStamped> positions;
  positions.reserve(raw_navsat_.size() + raw_odom_.size());

  // NavSatFix → local Cartesian (+ optional TF lever-arm offset).
  for (const auto & r : raw_navsat_) {
    if (!proj_.has_value()) {
      proj_.emplace(r.lat, r.lon, r.alt, ellipsoid_);
      RCLCPP_INFO(
        logger_, "NavSatFix origin (first fix): %f %f %f", r.lat, r.lon, r.alt);
    }
    double x, y, z;
    proj_->Forward(r.lat, r.lon, r.alt, x, y, z);
    Eigen::Vector3d p(x, y, z);
    if (!target_frame_.empty() && !r.frame_id.empty()) {
      p += transform_offset(r.frame_id, to_tf2_time(r.stamp));
    }
    positions.emplace_back(
      PointStdDev(p.x(), p.y(), p.z(), r.x_stddev, r.y_stddev, r.z_stddev), to_nanosec(r.stamp));
  }

  // Odometry → already in cartesian frame_id; just optionally apply lever arm.
  for (const auto & r : raw_odom_) {
    Eigen::Vector3d p = r.pos;
    if (!target_frame_.empty() && !r.child_frame_id.empty()) {
      p += transform_offset(r.child_frame_id, to_tf2_time(r.stamp));
    }
    positions.emplace_back(
      PointStdDev(p.x(), p.y(), p.z(), r.x_stddev, r.y_stddev, r.z_stddev), to_nanosec(r.stamp));
  }

  std::sort(
    positions.begin(), positions.end(),
    [](const PointStdDevStamped & a, const PointStdDevStamped & b) { return a.stamp < b.stamp; });

  if (skipped_stddev_ > 0) {
    RCLCPP_INFO(
      logger_, "Skipped %zu reference frames due to stddev threshold", skipped_stddev_);
  }
  RCLCPP_INFO(logger_, "Loaded %zu reference positions from bag", positions.size());
  return positions;
}
}  // namespace flexcloud
