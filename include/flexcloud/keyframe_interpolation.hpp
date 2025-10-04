/*
 * TUM Autonomous Motorsport Georeferencing Tool
 * Copyright (C) 2024 Maximilian Leitenstern, Marko Alten
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <eigen3/unsupported/Eigen/Splines>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "file_io.hpp"
#include "utility.hpp"
#include "visualization.hpp"
namespace flexcloud
{
class KeyframeInterpolation
{
public:
  KeyframeInterpolation(
    const std::string & config_path, const std::string & pos_dir, const std::string & odom_path,
    const std::string & dst_directory);
  KeyframeInterpolation(
    const std::string & config_path, const std::string & pos_dir, const std::string & odom_path,
    const std::string & pcd_dir, const std::string & dst_directory);
  void visualize();

private:
  /**
   * @brief Load frames from a directory
   *
   * @param[in] pos_dir             - std::string:
   *                                  absolute path to directory
   * @param[in] kitti_path          - std::string:
   *                                  path to kitti odometry
   */
  void load(
    const std::string & pos_dir, const std::string & odom_path);
  /**
   * @brief Load frames from a directory
   *
   * @param[in] pos_dir             - std::string:
   *                                  absolute path to directory
   * @param[in] kitti_path          - std::string:
   *                                  path to kitti odometry
   * @param[in] pcd_dir             - std::string:
   *                                  absolute path to directory
   */
  void load(
    const std::string & pos_dir, const std::string & odom_path,
    const std::string & pcd_dir);
  /**
   * @brief Save everything to directory
   *
   * @param[in] dst_directory       - std::string:
   *                                  absolute path to directory
   */
  bool save(const std::string & dst_directory) const;
  /**
   * @brief Select keyframes
   */
  void select_keyframes();
  /**
   * @brief Search closest PointStdDevStamped for a given frame
   *
   * @param[in] frame              - std::shared_ptr<OdometryFrame>:
   *                                 frame to search for
   * @return PointStdDevStamped    - PointStdDevStamped:
   *                                 closest PointStdDevStamped
   */
  PointStdDevStamped search_closest(const std::shared_ptr<OdometryFrame> & frame);
  /**
   * @brief Interpolate PointStdDevStamped for a given frame
   *
   * @param[in] frame              - std::shared_ptr<OdometryFrame>:
   *                                 frame to interpolate
   * @return PointStdDevStamped     - PointStdDevStamped:
   *                                 interpolated PointStdDevStamped
   */
  PointStdDevStamped interpolate_pos(const std::shared_ptr<OdometryFrame> & frame);
  /**
   * @brief Set parameters from config
   */
  void set_params(const std::string & config_path);

private:
  // File IO
  std::shared_ptr<file_io> file_io_;

  // Data
  std::vector<std::shared_ptr<OdometryFrame>> frames_;
  std::vector<PointStdDevStamped> pos_frames_;
  std::vector<std::shared_ptr<OdometryFrame>> keyframes_;
  std::vector<PointStdDevStamped> pos_keyframes_;

  // Visualization
  std::shared_ptr<visualization> viz_;
  rerun::RecordingStream rec_ = rerun::RecordingStream("keyframe_interpolation");

  // Config parameters
  std::string odom_format_{};
  float stddev_threshold_{1.0f};
  float keyframe_delta_x_{1.0f};
  float keyframe_delta_angle_{10.0f};
  float downsample_resolution_{0.1f};
  bool interpolate_{false};
  float pos_delta_xyz_{1.0f};
  std::int64_t globalMaxTimeDiff_{};
};
}  // namespace flexcloud
