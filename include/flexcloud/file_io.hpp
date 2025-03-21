/*
 * TUM Autonomous Motorsport Georeferencing Tool
 * Copyright (C) 2024 Maximilian Leitenstern, Marko Alten, Christian Bolea-Schaser
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
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/NormalGravity.hpp>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "param_pcd_georef.hpp"
#include "utility.hpp"
namespace flexcloud
{
class file_io
{
public:
  // Constructor
  file_io() {}
  /**
   * @brief read traj from txt file
   *
   * @param[in] node                - rclcpp::Node:
   *                                  Node reference
   * @param[in] traj_path           - std::string:
   *                                  absolute path to file
   * @param[in] traj_local          - std::vector<ProjPoint>:
   *                                  trajectory as vector of positions with standard dev
   */
  bool read_traj_from_file(
    FlexCloudConfig & config, const std::string & traj_path, std::vector<ProjPoint> & traj_local);

  /**
   * @brief read poses from txt file in KITTI format
   *
   * @param[in] node                - rclcpp::Node:
   *                                  Node reference
   * @param[in] poses_path          - std::string:
   *                                  absolute path to file
   * @param[in] poses               - std::vector<ProjPoint>:
   *                                  trajectory as vector of positions with standard dev
   */
  bool read_poses_SLAM_from_file(
    FlexCloudConfig & config, const std::string & poses_path, std::vector<ProjPoint> & poses);

  /**
   * @brief read pcd map from file
   *
   * @param[in] node                - rclcpp::Node:
   *                                  Node reference
   * @param[in] pcd_path            - std::string:
   *                                  absolute path to file
   * @param[in] pcm                 - pcl::PointCloud<pcl::PointXYZ>::Ptr:
   *                                  pointer on pointcloud map
   */
  bool read_pcd_from_file(
    FlexCloudConfig & config, const std::string & pcd_path,
    pcl::PointCloud<pcl::PointXYZI>::Ptr & pcm);

  /**
   * @brief write pcd map to file
   *
   * @param[in] node                - rclcpp::Node:
   *                                  Node reference
   * @param[in] pcd_out_path        - std::string:
   *                                  absolute path to file
   * @param[in] pcm                 - pcl::PointCloud<pcl::PointXYZ>::Ptr:
   *                                  pointer on pointcloud map
   */
  bool write_pcd_to_path(
    const std::string & pcd_out_path, const pcl::PointCloud<pcl::PointXYZI>::Ptr & pcd_map);
};
}  // namespace flexcloud
