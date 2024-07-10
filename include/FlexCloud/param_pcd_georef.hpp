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

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
namespace FlexCloud
{

/**
 * @brief declare ros parameters from yaml file
 * 
 * @param[in] node                  - rclcpp::Node:
 *                                    reference to node 
 * @param[in] traj_path             - std::string:
 *                                    path to GNSS trajectory
 * @param[in] poses_path            - std::string:
 *                                    path to SLAM trajectory
 * @param[in] pcd_path              - std::string:
 *                                    path to pcd
 * @param[in] pcd_out_path          - std::string:
 *                                    path to write resulting pcd
 */
void get_params(
  rclcpp::Node & node, std::string & traj_path, std::string & poses_path, std::string & pcd_path,
  std::string & pcd_out_path)
{
  // Get parameters from command line and param file
  // Paths => command line arguments (default in launch file)
  node.declare_parameter<std::string>("traj_path");
  node.declare_parameter<std::string>("poses_path");
  node.declare_parameter<std::string>("pcd_path");
  node.declare_parameter<std::string>("pcd_out_path");
  node.get_parameter("traj_path", traj_path);
  node.get_parameter("poses_path", poses_path);
  node.get_parameter("pcd_path", pcd_path);
  node.get_parameter("pcd_out_path", pcd_out_path);

  // Transformation GPS - Poses
  node.declare_parameter<int>("dim");
  node.declare_parameter<bool>("transform_traj");
  node.declare_parameter<int>("rs_num_controlPoints");
  node.declare_parameter<double>("stddev_threshold");
  node.declare_parameter<std::vector<double>>("square_size");
  node.declare_parameter<bool>("transform_pcd");
  node.declare_parameter<bool>("save_ascii");
  node.declare_parameter<bool>("auto_cp");
  node.declare_parameter<std::vector<int64_t>>("exclude_ind");
  node.declare_parameter<std::vector<int64_t>>("shift_ind");
  node.declare_parameter<std::vector<double>>("shift_ind_dist");
  node.declare_parameter<bool>("use_threading");
  node.declare_parameter<int>("num_cores");

  // Zero point
  node.declare_parameter<bool>("customZeroPoint");
  node.declare_parameter<double>("zeroLat");
  node.declare_parameter<double>("zeroLon");
  node.declare_parameter<double>("zeroEle");

  // Analysis Output
  node.declare_parameter<std::string>("analysis_output_dir");
  node.declare_parameter<bool>("analysis_traj_matching");
}
}  // namespace FlexCloud
