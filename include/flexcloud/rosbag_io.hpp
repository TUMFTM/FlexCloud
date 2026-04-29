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
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <vector>

#include "rosbag_reader.hpp"
#include "utility.hpp"

namespace flexcloud
{
/**
 * @brief Reads reference position data from a ROS 2 bag in a single pass.
 *
 * Supports `sensor_msgs/msg/NavSatFix` and `nav_msgs/msg/Odometry`. The
 * constructor opens the bag once and registers listeners on the embedded
 * `tools::RosbagReader` for `/tf`, `/tf_static`, and the configured topic;
 * `run()` triggers a single `reader_.process()` pass. While the bag is being
 * iterated, the TF buffer is populated and the position messages are queued
 * in their raw form. After the pass finishes, the queued messages are
 * transformed into `target_frame` using the **translation from the static**
 * transform (lever arm) and the **orientation from the dynamic** transform
 * at the message timestamp (mirroring rosbag_to_gnss in iac_map_loc) and
 * returned as `PointStdDevStamped`.
 */
class rosbag_io
{
public:
  /**
   * @param[in] bag_path          path to the bag (mcap or sqlite3 directory)
   * @param[in] topic             name of the position topic
   * @param[in] target_frame      TF frame to transform positions into; empty disables TF
   * @param[in] stddev_threshold  drop frames with horizontal stddev exceeding this value
   * @param[in] origin            optional [lat, lon, alt] for NavSatFix → ENU projection;
   *                              if std::nullopt, the first valid fix is used as the origin
   */
  rosbag_io(
    const std::string & bag_path, const std::string & topic, const std::string & target_frame,
    float stddev_threshold,
    const std::optional<Eigen::Vector3d> & origin = std::nullopt);

  /// Iterate the bag once and return the resulting positions, sorted by timestamp.
  std::vector<PointStdDevStamped> run();

protected:
  void tf_callback(const tools::RosbagReaderMsg<tf2_msgs::msg::TFMessage> & msg);
  void navsatfix_callback(const tools::RosbagReaderMsg<sensor_msgs::msg::NavSatFix> & msg);
  void odometry_callback(const tools::RosbagReaderMsg<nav_msgs::msg::Odometry> & msg);

private:
  /// Resolve the configured topic's message type from the bag metadata.
  std::string resolve_topic_type();

  /// World-frame offset to add to a message-frame position so that the result
  /// is the position of `target_frame_` in world coordinates.
  Eigen::Vector3d transform_offset(
    const std::string & src_frame, const tf2::TimePoint & stamp) const;

  // Configuration
  std::string bag_path_;
  std::string topic_;
  std::string topic_type_;
  std::string target_frame_;
  float stddev_threshold_;
  std::optional<Eigen::Vector3d> origin_;

  // ROS state
  tools::RosbagReader reader_;
  tf2::BufferCore tf_buffer_;
  GeographicLib::Geocentric ellipsoid_;
  std::optional<GeographicLib::LocalCartesian> proj_;
  rclcpp::Logger logger_;

  // Per-message raw data, populated by the topic callbacks during process().
  // Transformation is deferred until after the pass so that the full TF tree
  // (both static and dynamic) is available for the lookup.
  struct RawNavSatFix
  {
    double lat, lon, alt;
    double x_stddev, y_stddev, z_stddev;
    std::string frame_id;
    builtin_interfaces::msg::Time stamp;
  };
  struct RawOdom
  {
    Eigen::Vector3d pos;
    double x_stddev, y_stddev, z_stddev;
    std::string child_frame_id;
    builtin_interfaces::msg::Time stamp;
  };
  std::vector<RawNavSatFix> raw_navsat_{};
  std::vector<RawOdom> raw_odom_{};

  // Stats
  std::size_t n_static_tf_{0};
  std::size_t n_dynamic_tf_{0};
  std::size_t skipped_stddev_{0};
};
}  // namespace flexcloud
