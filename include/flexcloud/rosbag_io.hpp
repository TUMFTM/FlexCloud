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
#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <chrono>
#include <memory>
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
 * constructor opens the bag once, registers listeners on the embedded
 * `tools::RosbagReader` for `/tf`, `/tf_static` and the configured topic,
 * and sets up the geoid + ENU projection (NavSatFix only). `run()` triggers a
 * single `reader_.process()` pass; each callback projects / transforms the
 * message inline and pushes the resulting `PointStdDevStamped` directly into
 * `positions_`. Only one of the two message types is used per instance.
 *
 * NavSatFix → local ENU uses GeographicLib's `egm2008-2_5` geoid model: the
 * geoid height at the chosen origin is subtracted from each fix's altitude
 * before projection, so that the ENU origin lies on the geoid surface (mean
 * sea level approximation) rather than on the WGS84 ellipsoid.
 *
 * If a target frame is given, the message position is transformed via a
 * single `lookupTransform(target_frame, msg_frame, msg_stamp)` — tf2 walks
 * the entire static + dynamic chain at the message timestamp and returns the
 * fully composed transform; no separate static/dynamic queries are needed.
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

  /// Single-lookup TF resolution: returns the composite transform `target ← src`
  /// at the message timestamp (covers static and dynamic edges in one shot).
  /// Returns identity (and logs a warning) if the buffer can't satisfy the lookup.
  Eigen::Isometry3d lookup(const std::string & src_frame, const tf2::TimePoint & stamp) const;

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
  GeographicLib::Geocentric ellipsoid_;
  std::optional<GeographicLib::LocalCartesian> proj_;
  double geoid_height_{0.0};
  rclcpp::Logger logger_;

  // Output, populated inline by the topic callback
  std::vector<PointStdDevStamped> positions_;

  // Stats
  std::size_t n_static_tf_{0};
  std::size_t n_dynamic_tf_{0};
  std::size_t skipped_stddev_{0};
};
}  // namespace flexcloud
