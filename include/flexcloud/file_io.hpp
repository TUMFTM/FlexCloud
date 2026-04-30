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

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "cli/cli_config.hpp"
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
   * @param [in] origin             - std::optional<Eigen::Vector3d>:
   *                                 optional origin for ENU projection
   * @return std::vector<PointStdDevStamped>:
   *                                 vector of frames
   */
  std::vector<PointStdDevStamped> load_positions_dir(
    const std::string & directory, const float stddev_threshold,
    const std::optional<Eigen::Vector3d> & origin = std::nullopt);
  /**
   * @brief Load positions from a single txt file with one position per line.
   * 
   * @param[in] file_path          - std::string:
   *                                  absolute path to file
   * @param[in] stddev_threshold    - float:
   *                                  threshold for standard deviation
   * @param [in] origin             - std::optional<Eigen::Vector3d>:
   *                                  optional origin for ENU projection
   * @return std::vector<PointStdDevStamped>:
   *                                  vector of position frames
   */
  std::vector<PointStdDevStamped> load_positions_file(
    const std::string & file_path, const float stddev_threshold,
    const std::optional<Eigen::Vector3d> & origin = std::nullopt);
  /**
   * @brief Load glim odometry from a file
   *
   * @param[in] file_path           - std::string:
   *                                 absolute path to file
   * @return std::vector<Eigen::Isometry3d>:
   *                                 vector of poses
   */
  std::vector<PoseStamped> load_poses(const std::string & file_path);

  /**
   * @brief Read a PCD map from file into a `pcl::PCLPointCloud2`. All fields
   *        present in the file are preserved.
   * 
   * @param[in] pcd_path            - std::string:
   *                                  absolute path to PCD file
   * @param[out] pcm                - pcl::PCLPointCloud2::Ptr:
   *                                  point cloud map; allocated and filled by this function
   * @return true if executed
   */
  bool load_pcd(const std::string & pcd_path, pcl::PCLPointCloud2::Ptr & pcm);

  /**
   * @brief save position frames to file
   *
   * @param[in] filename            - std::string:
   *                                  absolute path to file
   * @param[in] keyframes           - std::vector<std::shared_ptr<PoseStamped>>:
   *                                  vector of keyframes
   */
  bool save_positions(
    const std::string & filename, const std::vector<PointStdDevStamped> & positions);
  /**
   * @brief save kitti odometry to file
   *
   * @param[in] filename            - std::string:
   *                                  absolute path to file
   * @param[in] keyframes           - std::vector<std::shared_ptr<OdometryFrame>>:
   *                                  vector of keyframes
   */
  bool save_poses(const std::string & filename, const std::vector<PoseStamped> & poses);

  /**
   * @brief Write a `pcl::PCLPointCloud2` map to file
   * 
   * @param[in] pcd_out_path        - std::string:
   *                                  absolute path to output PCD file
   * @param[in] pcd_map             - pcl::PCLPointCloud2::Ptr:
   *                                  point cloud map to save
   * @return true if executed
   */
  bool save_pcd(const std::string & pcd_out_path, const pcl::PCLPointCloud2::Ptr & pcd_map);

private:
};
}  // namespace flexcloud
