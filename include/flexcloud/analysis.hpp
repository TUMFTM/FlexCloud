
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

#include "param_pcd_georef.hpp"
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
   * @param[in] config              - FlexCloudConfig:
   *                                  config struct
   * @param[in] src                 - std::vector<PointStdDev>:
   *                                  source trajectory
   * @param[in] target              - std::vector<PointStdDev>:
   *                                  target trajectory
   * @param[in] target_al           - std::vector<PointStdDev>:
   *                                  target trajectory after Umeyama trafo
   * @param[in] target_rs           - std::vector<PointStdDev>:
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
    FlexCloudConfig & config, const std::vector<PointStdDev> & src,
    const std::vector<PointStdDev> & target, const std::vector<PointStdDev> & target_al,
    const std::vector<PointStdDev> & target_rs, const std::shared_ptr<Delaunay> & triag,
    const std::vector<ControlPoint> & cps, std::vector<double> & diff_al,
    std::vector<double> & diff_rs);

private:
  /**
   * @brief calculate difference of a target trajectory to a source trajectory
   *
   * @param[in] config              - FlexCloudConfig:
   *                                  config struct
   * @param[in] src                 - std::vector<PointStdDev>:
   *                                  source trajectory
   * @param[in] target              - std::vector<PointStdDev>:
   *                                  target trajectory
   * @param[in] diff                - std::vector<double>:
   *                                  difference between trajectories (euclidean distance)
   */
  void calc_diff(
    FlexCloudConfig & config, const std::vector<PointStdDev> & src,
    const std::vector<PointStdDev> & target, std::vector<double> & diff);
  /**
   * @brief Save FlexCloudConfig to a text file
   * @param config The configuration to save
   * @param filepath Path to save the configuration file
   * @return true if successful, false otherwise
   */
  void save_config(
    const FlexCloudConfig & config, const std::string & dir_path, const std::string & file_name);
  /**
   * @brief write a linestring to .txt file
   *
   * @param[in] ls                  - std::vector<PointStdDev>:
   *                                  linestring
   * @param[in] dir_path            - std::string:
   *                                  name of output directory
   * @param[in] file_name           - std::string:
   *                                  name of output file
   */
  void write_ls(const std::vector<PointStdDev> & ls, const std::string & dir_path,
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
  void write_lss(const std::vector<std::vector<PointStdDev>> & lss,
    const std::string & dir_path, const std::string & file_name);
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
  void write_double_vec(const std::vector<double> & vec, const std::string & dir_path,
    const std::string & file_name);

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
  void write_triag(const std::shared_ptr<Delaunay> & triag, const std::string & dir_path,
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
  void write_cp(const std::vector<ControlPoint> & cps, const std::string & dir_path,
    const std::string & file_name);
};
}  // namespace flexcloud
