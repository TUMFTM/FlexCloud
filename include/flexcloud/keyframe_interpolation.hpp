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

#include "cli/cli_config.hpp"
#include "file_io.hpp"
#include "rosbag_io.hpp"
#include "utility.hpp"
#include "visualization.hpp"
namespace flexcloud
{
class KeyframeInterpolation
{
public:
  explicit KeyframeInterpolation(const config::KeyframeInterpolationConfig & cli_cfg);
  void visualize();

private:
  /**
   * @brief Load reference positions and SLAM poses based on the CLI configuration.
   */
  void load(const config::KeyframeInterpolationConfig & cli_cfg);
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
  void select_keyframes(const config::KeyframeInterpolationConfig & cli_cfg);
  /**
   * @brief Search closest PointStdDevStamped for a given frame
   *
   * @param[in] frame              - PoseStamped:
   *                                 frame to search for
   * @return PointStdDevStamped    - PointStdDevStamped:
   *                                 closest PointStdDevStamped
   */
  PointStdDevStamped search_closest(const PoseStamped & frame);
  /**
   * @brief Interpolate PointStdDevStamped for a given frame
   *
   * @param[in] frame              - PoseStamped:
   *                                 frame to interpolate
   * @return PointStdDevStamped     - PointStdDevStamped:
   *                                 interpolated PointStdDevStamped
   */
  PointStdDevStamped interpolate_pos(const PoseStamped & frame, const double pos_delta_xyz);

private:
  // File IO
  std::shared_ptr<file_io> file_io_{std::make_shared<file_io>()};

  // Data
  std::vector<PoseStamped> poses_;
  std::vector<PointStdDevStamped> positions_;
  std::vector<PoseStamped> key_poses_;
  std::vector<PointStdDevStamped> key_positions_;
  std::int64_t max_time_diff_{};


  // Visualization
  std::shared_ptr<visualization> viz_{std::make_shared<visualization>()};
  rerun::RecordingStream rec_ = rerun::RecordingStream("keyframe_interpolation");
};
}  // namespace flexcloud
