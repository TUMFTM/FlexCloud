
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

#include <Eigen/Dense>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "triangulation.hpp"
#include "utility.hpp"
namespace tam::mapping
{
class analysis
{
public:
  // Constructor
  analysis() {}
  /**
   * @brief write all data relevant for evaluation of trajectory matching
   *
   * @param[in] node                - rclcpp::Node:
   *                                  Node reference
   * @param[in] src                 - std::vector<ProjPoint>:
   *                                  source trajectory
   * @param[in] target              - std::vector<ProjPoint>:
   *                                  target trajectory
   * @param[in] target_al           - std::vector<ProjPoint>:
   *                                  target trajectory after Umeyama trafo
   * @param[in] target_rs           - std::vector<ProjPoint>:
   *                                  target trajectory after rubber-sheeting
   * @param[in] triag               - std::shared_ptr<Delaunay>:
   *                                  pointer to triangulation
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  vector of control points
   * @param[in] diff_al             - std::vector<double>:
   *                                  difference of aligned trajectory to source trajectory
   * @param[in] diff_rs             - std::vector<double>:
   *                                  difference of rubber-sheeted trajectory to source trajectory
   */
  bool traj_matching(
    rclcpp::Node & node, const std::vector<ProjPoint> & src, const std::vector<ProjPoint> & target,
    const std::vector<ProjPoint> & target_al, const std::vector<ProjPoint> & target_rs,
    const std::shared_ptr<Delaunay> & triag, const std::vector<ControlPoint> & cps,
    std::vector<double> & diff_al, std::vector<double> & diff_rs);

private:
  /**
   * @brief calculate difference of a target trajectory to a source trajectory
   *
   * @param[in] node                - rclcpp::Node:
   *                                  Node reference
   * @param[in] src                 - std::vector<ProjPoint>:
   *                                  source trajectory
   * @param[in] target              - std::vector<ProjPoint>:
   *                                  target trajectory
   * @param[in] diff                - std::vector<double>:
   *                                  difference between trajectories (euclidean distance)
   */
  void calc_diff(
    rclcpp::Node & node, const std::vector<ProjPoint> & src, const std::vector<ProjPoint> & target,
    std::vector<double> & diff);

  /**
   * @brief create directory from name
   *
   * @param[in] node                - rclcpp::Node:
   *                                  Node reference
   * @param[in] dir_path            - std::string:
   *                                  name of output directory
   */
  void create_output_dir(rclcpp::Node & node, const std::string & dir_path);

  /**
   * @brief write a linestring to .txt file
   *
   * @param[in] node                - rclcpp::Node:
   *                                  Node reference
   * @param[in] ls                  - std::vector<ProjPoint>:
   *                                  linestring
   * @param[in] dir_path            - std::string:
   *                                  name of output directory
   * @param[in] file_name           - std::string:
   *                                  name of output file
   */
  void write_ls(
    rclcpp::Node & node, const std::vector<ProjPoint> & ls, const std::string & dir_path,
    const std::string & file_name);
  /**
   * @brief write a linestrings to .txt file
   *
   * @param[in] node                - rclcpp::Node:
   *                                  Node reference
   * @param[in] lss                 - std::vector<std::vector<ProjPoint>>:
   *                                  vector of linestrings
   * @param[in] dir_path            - std::string:
   *                                  name of output directory
   * @param[in] file_name           - std::string:
   *                                  name of output file
   */
  void write_lss(
    rclcpp::Node & node, const std::vector<std::vector<ProjPoint>> & lss,
    const std::string & dir_path, const std::string & file_name);
  /**
   * @brief write a double vector to .txt file
   *
   * @param[in] node                - rclcpp::Node:
   *                                  Node reference
   * @param[in] vec                 - std::vector<double>:
   *                                  vector of double values
   * @param[in] dir_path            - std::string:
   *                                  name of output directory
   * @param[in] file_name           - std::string:
   *                                  name of output file
   */
  void write_double_vec(
    rclcpp::Node & node, const std::vector<double> & vec, const std::string & dir_path,
    const std::string & file_name);

  /**
   * @brief write triangulation vertices to file
   *
   * @param[in] node                - rclcpp::Node:
   *                                  Node reference
   * @param[in] triag               - std::shared_ptr<Delaunay>:
   *                                  pointer to triangulation
   * @param[in] dir_path            - std::string:
   *                                  name of output directory
   * @param[in] file_name           - std::string:
   *                                  name of output file
   */
  void write_triag(
    rclcpp::Node & node, const std::shared_ptr<Delaunay> & triag, const std::string & dir_path,
    const std::string & file_name);
  /**
   * @brief write controlpoints to file
   *
   * @param[in] node                - rclcpp::Node:
   *                                  Node reference
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  control points
   * @param[in] dir_path            - std::string:
   *                                  name of output directory
   * @param[in] file_name           - std::string:
   *                                  name of output file
   */
  void write_cp(
    rclcpp::Node & node, const std::vector<ControlPoint> & cps, const std::string & dir_path,
    const std::string & file_name);
};
}  // namespace tam::mapping
