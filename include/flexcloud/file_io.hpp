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
#include <memory>
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
   * @brief Load position frames from a directory
   *
   * @param[in] directory           - std::string:
   *                                 absolute path to directory
   * @param[in] stddev_threshold    - float:
   *                                 threshold for standard deviation
   * @return std::vector<PosFrame>:
   *                                 vector of position frames
   */
  std::vector<PosFrame> load_pos_frames(
    const std::string & directory, const float stddev_threshold);
  /**
   * @brief Load kitti odometry from a file
   *
   * @param[in] file_path           - std::string:
   *                                 absolute path to file
   * @return std::vector<Eigen::Isometry3d>:
   *                                 vector of poses
   */
  std::vector<Eigen::Isometry3d> load_kitti_odom(const std::string & file_path);
  /**
   * @brief Load glim odometry from a file
   *
   * @param[in] file_path           - std::string:
   *                                 absolute path to file
   * @return std::vector<Eigen::Isometry3d>:
   *                                 vector of poses
   */
  std::vector<Eigen::Isometry3d> load_glim_odom(const std::string & file_path);
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
   * @brief Load pcd point clouds from a directory
   */
  std::vector<std::string> load_clouds(const std::string & directory);

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
   * @brief save kitti odometry to file
   *
   * @param[in] filename            - std::string:
   *                                  absolute path to file
   * @param[in] keyframes           - std::vector<std::shared_ptr<OdometryFrame>>:
   *                                  vector of keyframes
   */
  bool save_graph(
    const std::string & filename, const std::vector<std::shared_ptr<OdometryFrame>> & keyframes);
  /**
   * @brief save kitti odometry to file
   *
   * @param[in] filename            - std::string:
   *                                  absolute path to file
   * @param[in] keyframes           - std::vector<std::shared_ptr<OdometryFrame>>:
   *                                  vector of keyframes
   */
  bool save_kitti(
    const std::string & filename, const std::vector<std::shared_ptr<OdometryFrame>> & keyframes);
  /**
   * @brief save kitti odometry to file
   *
   * @param[in] filename            - std::string:
   *                                  absolute path to file
   * @param[in] keyframes           - std::vector<std::shared_ptr<OdometryFrame>>:
   *                                  vector of keyframes
   */
  bool save_keyframes(
    const std::string & directory, const std::vector<std::shared_ptr<OdometryFrame>> & keyframes,
    const float downsample);
  /**
   * @brief save position frames to file
   *
   * @param[in] filename            - std::string:
   *                                  absolute path to file
   * @param[in] pos_keyframes       - std::vector<PosFrame>:
   *                                  vector of position frames
   */
  bool save_pos_frames(const std::string & filename, const std::vector<PosFrame> & pos_keyframes);

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
