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

#include <iostream>
#include <memory>
#include <rerun.hpp>
#include <string>
#include <utility>
#include <vector>

#include "analysis.hpp"
#include "file_io.hpp"
#include "point_types.hpp"
#include "transform.hpp"
#include "visualization.hpp"
#include "yaml-cpp/yaml.h"
namespace flexcloud
{
/**
 * @brief basic class for flexcloud::Georeferencing package
 */
class Georeferencing
{
public:
  // Georeferencing package constructor
  Georeferencing(
    const std::string & config_path, const std::string & pos_global_path,
    const std::string & poses_path, const std::string & pcd_path);
  // Functions
  /**
   * @brief check if all necessary paths exist
   */
  bool paths_valid();

  /**
   * @brief load trajectories and pcd map
   */
  void load_data();

  /**
   * @brief align trajectories with Umeyama algorithm
   */
  void align_traj();

  /**
   * @brief publish resulting trajectories from alignment
   */
  void visualize_traj();

  /**
   * @brief apply automatic or manual rubber-sheet trafo and transform map
   *        and trajectories
   */
  void rubber_sheeting();

  /**
   * @brief visualize results from rubber-sheeting including transformed map
   */
  void visualize_rs();

  /**
   * @brief write pcd map to file
   */
  void save_map();

  /**
   * @brief do evaluation calculations and write to txt-files
   *
   * @param[in] config               - YAML::Node:
   *                                  configuration node
   */
  void evaluation(const YAML::Node & config);

private:
  // Config
  GeoreferencingConfig config_{};

  // Module classes
  transform transform_{};
  std::shared_ptr<file_io> file_io_ = std::make_shared<file_io>();
  std::shared_ptr<visualization> viz_ = std::make_shared<visualization>();
  std::shared_ptr<analysis> analysis_ = std::make_shared<analysis>();

  // Objects
  std::vector<PointStdDevStamped> pos_global_{};
  std::vector<PoseStamped> poses_{};
  std::vector<PoseStamped> poses_align_{};
  std::vector<PoseStamped> poses_rs_{};
  // PCD map
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_map_{};
  pcl::PointCloud<PointXYZIL>::Ptr pcd_map_il_{};

  // Transformation
  std::vector<ControlPoint> control_points_;
  std::shared_ptr<Umeyama> umeyama_ = std::make_shared<Umeyama>();
  std::shared_ptr<Delaunay> triag_ = std::make_shared<Delaunay>();

  // Visualization
  rerun::RecordingStream rec_ = rerun::RecordingStream("flexcloud");
};
}  // namespace flexcloud
