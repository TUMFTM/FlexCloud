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

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <algorithm>
#include <iostream>
#include <memory>
#include <rerun.hpp>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "triangulation.hpp"
#include "utility.hpp"
#include "visualization.hpp"
namespace flexcloud
{
class visualization
{
public:
  visualization() {}
  /**
   * @brief visualize linestring in rerun
   *
   * @param[in] ls                  - std::vector<ProjPoint>:
   *                                  controlpoints
   * @param[in] stream              - rerun::RecordingStream:
   *                                  stream to add linestring to
   * @param[in] color               - std_msgs::msg::ColorRGBA:
   *                                  color of array
   * @param[in] name                - std::string:
   *                                  namespace of linestring
   */
  void linestring2rerun(
    const std::vector<ProjPoint> & ls, rerun::RecordingStream & stream, const std::string color,
    const std::string name);
  /**
   * @brief visualize rubber-sheeting geometry in rerun
   *
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  vector of control points
   * @param[in] triag               - std::shared_ptr<Delaunay>:
   *                                  pointer to triangulation
   * @param[in] stream              - rerun::RecordingStream:
   *                                  stream to add linestring to
   * @param[in] color               - std::string:
   *                                  color of array
   * @param[in] name                - std::string:
   *                                  namespace of linestring
   */
  void rs2rerun(
    const std::vector<ControlPoint> & cps, std::shared_ptr<Delaunay> & triag,
    rerun::RecordingStream & stream, const std::string color);
  /**
   * @brief visualize point cloud map in rerun
   *
   * @param[in] pcd_map             - pcl::PointCloud<pcl::PointXYZ>::Ptr:
   *                                  pcd map
   * @param[in] stream              - rerun::RecordingStream:
   *                                  stream to add map to
   */
  void pc_map2rerun(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & pcd_map, rerun::RecordingStream & stream);
};
}  // namespace flexcloud
