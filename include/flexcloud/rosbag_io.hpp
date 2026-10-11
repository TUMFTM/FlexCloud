/*
 * TUM Autonomous Motorsport Georeferencing Tool
 * Copyright (C) 2026 Maximilian Leitenstern
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#pragma once

#include <tf2_ros/buffer.h>

#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rosbag_reader.hpp"
#include "utility.hpp"

namespace flexcloud
{
/**
 * @brief Reads reference position data from a ROS 2 bag in a single pass.
 *
 * Supports `sensor_msgs/msg/NavSatFix` and `nav_msgs/msg/Odometry`. TF lookups are
 * resolved after the pass, once every `/tf` and `/tf_static` message is buffered.
 */
class rosbag_io
{
public:
  /**
   * @param[in] bag_path          path to the bag (mcap or sqlite3 directory)
   * @param[in] topic             name of the position topic
   * @param[in] target_frame      frame of the SLAM poses (e.g. base_link) to move the positions
   *                              into, using the TF chain of the bag; empty disables TF
   * @param[in] stddev_threshold  drop frames with horizontal stddev exceeding this value
   * @param[in] origin            optional [lat, lon, alt] for NavSatFix → ENU projection, alt
   *                              above mean sea level; if std::nullopt, the first accepted fix
   *                              is used as the origin
   */
  rosbag_io(
    const std::string & bag_path, const std::string & topic, const std::string & target_frame,
    float stddev_threshold,
    const std::optional<Eigen::Vector3d> & origin = std::nullopt);

  /// Iterate the bag once and return the resulting positions, sorted by timestamp.
  std::vector<PointStdDevStamped> run();

  /// Origin [lat, lon, alt above mean sea level] of the ENU frame (custom or first accepted fix,
  /// valid after run()); std::nullopt for Odometry topics, which are not projected.
  const std::optional<Eigen::Vector3d> & origin() const { return origin_; }

protected:
  void tf_callback(const tools::RosbagReaderMsg<tf2_msgs::msg::TFMessage> & msg);
  void navsatfix_callback(const tools::RosbagReaderMsg<sensor_msgs::msg::NavSatFix> & msg);
  void odometry_callback(const tools::RosbagReaderMsg<nav_msgs::msg::Odometry> & msg);

private:
  /// Resolve the configured topic's message type from the bag metadata.
  std::string resolve_topic_type();

  /// Move the positions to target_frame (if set): the lever arm source → target is looked up in
  /// the world frame at each message stamp, so the vehicle orientation comes from TF. Positions
  /// whose lookup fails are dropped and counted.
  void resolve_positions();

  /// World frame of @p frame: its topmost parent in the TF tree. Throws if it has no parent or
  /// only static transforms lead there (then the root is part of the vehicle, not the world).
  std::string infer_world_frame(const std::string & frame) const;

  // Configuration
  std::string bag_path_;
  std::string topic_;
  std::string topic_type_;
  std::string target_frame_;
  float stddev_threshold_;
  std::optional<Eigen::Vector3d> origin_;

  // ROS / projection state
  tools::RosbagReader reader_;
  tf2::BufferCore tf_buffer_;
  std::unordered_map<std::string, std::string> tf_parents_;  // child frame → parent frame
  std::unordered_set<std::string> tf_moving_frames_;         // child frames of /tf transforms
  ENUProjection proj_;
  rclcpp::Logger logger_;

  // Frames of the topic, taken from its first message (assumed constant within the bag)
  std::string source_frame_;  // Odometry: child_frame_id, NavSatFix: header.frame_id (antenna)
  std::string world_frame_;   // Odometry: header.frame_id, NavSatFix: empty (TF root is used)

  // Output of by the topic callbacks and moved by resolve_positions()
  std::vector<PointStdDevStamped> positions_;

  // Stats
  std::size_t n_static_tf_{0};
  std::size_t n_dynamic_tf_{0};
  std::size_t skipped_no_fix_{0};
  std::size_t skipped_stddev_{0};
  std::size_t skipped_tf_{0};
};
}  // namespace flexcloud
