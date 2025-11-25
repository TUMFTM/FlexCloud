
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
#include <filesystem>  // NOLINT
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "triangulation.hpp"
#include "utility.hpp"
namespace flexcloud
{
class analysis
{
public:
  // Constructor
  analysis() {}
  /**
   * @brief write all data relevant for evaluation of trajectory matching
   *
   * @param[in] dir                 - std::string:
   *                                  name of output directory
   * @param[in] src                 - std::vector<PointStdDevStamped>:
   *                                  source trajectory
   * @param[in] target              - std::vector<PoseStamped>:
   *                                  target trajectory
   * @param[in] target_al           - std::vector<PoseStamped>:
   *                                  target trajectory after Umeyama trafo
   * @param[in] target_rs           - std::vector<PoseStamped>:
   *                                  target trajectory after rubber-sheeting
   * @param[in] triag               - std::shared_ptr<Delaunay>:
   *                                  pointer to triangulation
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  vector of control points
   */
  bool traj_matching(
    const std::string & dir, const std::vector<PointStdDevStamped> & src,
    const std::vector<PoseStamped> & target, const std::vector<PoseStamped> & target_al,
    const std::vector<PoseStamped> & target_rs, const std::shared_ptr<Delaunay> & triag,
    const std::vector<ControlPoint> & cps);

private:
  /**
   * @brief calculate difference of a target trajectory to a source trajectory
   *
   * @param[in] src                 - std::vector<PointStdDevStamped>:
   *                                  source trajectory
   * @param[in] target              - std::vector<PoseStamped>:
   *                                  target trajectory
   * @return std::vector<double>    - std::vector<double>:
   *                                  difference between trajectories (euclidean distance)
   */
  std::vector<double> calc_diff(
    const std::vector<PointStdDevStamped> & src, const std::vector<PoseStamped> & target);
  /**
   * @brief write a linestring to .txt file
   *
   * @param[in] ls                  - std::vector<PointStdDevStamped>:
   *                                  linestring
   * @param[in] dir_path            - std::string:
   *                                  name of output directory
   * @param[in] file_name           - std::string:
   *                                  name of output file
   */
  void write_ls(
    const std::vector<PointStdDevStamped> & ls, const std::string & dir_path,
    const std::string & file_name);
  /**
   * @brief write a linestring to .txt file
   *
   * @param[in] ls                  - std::vector<PoseStamped>:
   *                                  linestring
   * @param[in] dir_path            - std::string:
   *                                  name of output directory
   * @param[in] file_name           - std::string:
   *                                  name of output file
   */
  void write_ls(
    const std::vector<PoseStamped> & ls, const std::string & dir_path,
    const std::string & file_name);
  /**
   * @brief write a linestrings to .txt file
   *
   * @param[in] node                - rclcpp::Node:
   *                                  Node reference
   * @param[in] lss                 - std::vector<std::vector<PointStdDev>>:
   *                                  vector of linestrings
   * @param[in] dir_path            - std::string:
   *                                  name of output directory
   * @param[in] file_name           - std::string:
   *                                  name of output file
   */
  void write_lss(
    const std::vector<std::vector<PointStdDev>> & lss, const std::string & dir_path,
    const std::string & file_name);
  /**
   * @brief write a double vector to .txt file
   *
   * @param[in] vec                 - std::vector<double>:
   *                                  vector of double values
   * @param[in] dir_path            - std::string:
   *                                  name of output directory
   * @param[in] file_name           - std::string:
   *                                  name of output file
   */
  void write_double_vec(
    const std::vector<double> & vec, const std::string & dir_path, const std::string & file_name);

  /**
   * @brief write triangulation vertices to file
   *
   * @param[in] triag               - std::shared_ptr<Delaunay>:
   *                                  pointer to triangulation
   * @param[in] dir_path            - std::string:
   *                                  name of output directory
   * @param[in] file_name           - std::string:
   *                                  name of output file
   */
  void write_triag(
    const std::shared_ptr<Delaunay> & triag, const std::string & dir_path,
    const std::string & file_name);
  /**
   * @brief write controlpoints to file
   *
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  control points
   * @param[in] dir_path            - std::string:
   *                                  name of output directory
   * @param[in] file_name           - std::string:
   *                                  name of output file
   */
  void write_cp(
    const std::vector<ControlPoint> & cps, const std::string & dir_path,
    const std::string & file_name);
};
}  // namespace flexcloud
