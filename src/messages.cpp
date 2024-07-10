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

#include "messages.hpp"
namespace FlexCloud
{
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
void messages::linestring2marker_msg(
  const std::vector<ProjPoint> & ls, visualization_msgs::msg::MarkerArray & msg,
  const std::string color, const std::string ns)
{
  std_msgs::msg::ColorRGBA col;
  set_color(&col, color, 0.999);
  insert_marker_array(&msg, linestring2marker(ls, ns, col));
  insert_marker_array(&msg, ids2marker(ls, ns + "_ind", col));
}
/**
 * @brief add triangulation to marker msg
 *
 * @param[in] triag               - std::shared_ptr<Delaunay>:
 *                                  pointer to triangulation
 * @param[in] msg                 - visualization_msgs::msg::MarkerArray:
 *                                  msg of marker array
 */
void messages::rs2marker_msg_tet(
  std::shared_ptr<Delaunay> & triag, visualization_msgs::msg::MarkerArray & msg)
{
  insert_marker_array(&msg, triag2marker(triag, "Ivory", "triag"));
}
/**
 * @brief add control points to marker msg
 *
 * @param[in] cps                 - std::vector<ControlPoint>:
 *                                  vector of control points
 * @param[in] msg                 - visualization_msgs::msg::MarkerArray:
 *                                  msg of marker array
 */
void messages::rs2marker_msg_cps(
  const std::vector<ControlPoint> & cps, visualization_msgs::msg::MarkerArray & msg)
{
  insert_marker_array(&msg, controlpoints2marker(cps, "WEBOrange", "controlpoint"));
}
/**
 * @brief convert pcd map to ros msg
 *
 * @param[in] pcd_map             - pcl::PointCloud<pcl::PointXYZ>::Ptr:
 *                                  pcd map
 * @param[in] msg                 - sensor_msgs::msg::PointCloud2:
 *                                  msg of pcd map
 */
void messages::pcd_map2msg(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & pcd_map, sensor_msgs::msg::PointCloud2 & msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> grid;
  grid.setInputCloud(pcd_map);
  grid.setLeafSize(1.5, 1.5, 1.5);
  grid.filter(*cloud_filtered);
  pcl::toROSMsg(*cloud_filtered, msg);
  msg.header.frame_id = "map";
}
}  // namespace FlexCloud
