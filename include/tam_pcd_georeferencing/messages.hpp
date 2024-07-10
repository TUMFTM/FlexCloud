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
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <algorithm>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "track_handler_cpp/race_track_handler.hpp"
#include "utility.hpp"
#include "visualization.hpp"
namespace tam::mapping
{
class messages
{
public:
  messages() {}

  /**
   * @brief add linestring to marker msg
   * 
   * @param[in] ls                  - std::vector<ProjPoint>:
   *                                  controlpoints
   * @param[in] msg                 - visualization_msgs::msg::MarkerArray:
   *                                  msg of marker array
   * @param[in] ns                  - std::string:
   *                                  namespace of markerarray
   * @param[in] c                   - std_msgs::msg::ColorRGBA:
   *                                  color of array
   */
  void linestring2marker_msg(
    const std::vector<ProjPoint> & ls, visualization_msgs::msg::MarkerArray & msg,
    const std::string color, const std::string ns);

  /**
   * @brief add trackbounds to marker msg
   * 
   * @param[in] ls                  - std::vector<std::vector<ProjPoint>>:
   *                                  vector of trackbounds (left, right, pit...)
   * @param[in] msg                 - visualization_msgs::msg::MarkerArray:
   *                                  msg of marker array
   * @param[in] ns                  - std::string:
   *                                  namespace of markerarray
   * @param[in] c                   - std_msgs::msg::ColorRGBA:
   *                                  color of array
   */
  void trackbounds2marker_msg(
    const std::vector<std::vector<ProjPoint>> & ls, visualization_msgs::msg::MarkerArray & msg,
    const std::string color, const std::string ns);

  /**
   * @brief add triangulation to marker msg
   * 
   * @param[in] triag               - std::shared_ptr<Delaunay>:
   *                                  pointer to triangulation
   * @param[in] msg                 - visualization_msgs::msg::MarkerArray:
   *                                  msg of marker array
   */
  void rs2marker_msg_tet(
    std::shared_ptr<Delaunay> & triag, visualization_msgs::msg::MarkerArray & msg);

  /**
   * @brief add control points to marker msg
   * 
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  vector of control points
   * @param[in] msg                 - visualization_msgs::msg::MarkerArray:
   *                                  msg of marker array
   */
  void rs2marker_msg_cps(
    const std::vector<ControlPoint> & cps, visualization_msgs::msg::MarkerArray & msg);

  /**
   * @brief convert pcd map to ros msg
   * 
   * @param[in] pcd_map             - pcl::PointCloud<pcl::PointXYZ>::Ptr:
   *                                  pcd map
   * @param[in] msg                 - sensor_msgs::msg::PointCloud2:
   *                                  msg of pcd map
   */
  void pcd_map2msg(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & pcd_map, sensor_msgs::msg::PointCloud2 & msg);

  /**
   * @brief add trackbounds to linestrings
   * 
   * @param[in] tr                  - std::unique_ptr<iac::common::Track>:
   *                                  pointer to trackhandler 
   * @param[in] ls_left             - std::vector<ProjPoint>:
   *                                  left track bound
   * @param[in] ls_right            - std::vector<ProjPoint>:
   *                                  right track bound
   */
  void track2ls(
    const std::unique_ptr<iac::common::Track> & tr, std::vector<ProjPoint> & ls_left,
    std::vector<ProjPoint> & ls_right);
};
}  // namespace tam::mapping
