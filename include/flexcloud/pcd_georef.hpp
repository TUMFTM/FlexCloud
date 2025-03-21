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
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>
#include <rerun.hpp>

#include "analysis.hpp"
#include "file_io.hpp"
#include "visualization.hpp"
#include "param_pcd_georef.hpp"
#include "transform.hpp"
namespace flexcloud
{
/**
 * @brief basic class for flexcloud::pcd_georef package
 */
class pcd_georef : public rclcpp::Node
{
public:
  // pcd_georef package constructor
  explicit pcd_georef(const std::string & name);
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
  void write_map();

  /**
   * @brief do evaluation calculations and write to txt-files
   */
  void evaluation();

private:
  std::string node_name;
  // User parameters
  std::string traj_path;
  std::string poses_path;
  std::string pcd_path;
  std::string pcd_out_path;

  // Module classes
  std::shared_ptr<file_io> file_io_;
  transform transform_;
  std::shared_ptr<visualization> viz_;
  std::shared_ptr<analysis> analysis_;

  // Objects
  // Trajectories
  std::vector<ProjPoint> traj_proj;
  std::vector<ProjPoint> traj_SLAM;
  std::vector<ProjPoint> traj_align;
  std::vector<ProjPoint> traj_rs;
  // PCD map
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_map;

  // Transformation
  std::vector<ControlPoint> control_points;
  std::shared_ptr<Umeyama> umeyama_;
  std::shared_ptr<Delaunay> triag_;

  // Visualization
  rerun::RecordingStream rec_ = rerun::RecordingStream("flexcloud");
};
}  // namespace flexcloud
