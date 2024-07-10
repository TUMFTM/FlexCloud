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
namespace tam::mapping
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
void messages::trackbounds2marker_msg(
  const std::vector<std::vector<ProjPoint>> & ls, visualization_msgs::msg::MarkerArray & msg,
  const std::string color, const std::string ns)
{
  std_msgs::msg::ColorRGBA col;
  set_color(&col, color, 0.999);

  insert_marker_array(&msg, linestring2marker(ls[0], ns + "_track_left", col));
  insert_marker_array(&msg, linestring2marker(ls[1], ns + "_track_right", col));
  insert_marker_array(&msg, linestring2marker(ls[2], ns + "_pit_left", col));
  insert_marker_array(&msg, linestring2marker(ls[3], ns + "_pit_right", col));
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
void messages::track2ls(
  const std::unique_ptr<iac::common::Track> & tr, std::vector<ProjPoint> & ls_left,
  std::vector<ProjPoint> & ls_right)
{
  const Eigen::VectorXd x_left = tr->left_bound_x();
  const Eigen::VectorXd y_left = tr->left_bound_y();
  const Eigen::VectorXd z_left = tr->left_bound_z();
  const Eigen::VectorXd x_right = tr->right_bound_x();
  const Eigen::VectorXd y_right = tr->right_bound_y();
  const Eigen::VectorXd z_right = tr->right_bound_z();

  ls_left.clear();
  ls_right.clear();
  for (int i = 0; i < static_cast<int>(x_left.size()); ++i) {
    ProjPoint pt(x_left[i], y_left[i], z_left[i], 0.0, 0.0, 0.0);
    ls_left.push_back(pt);
  }
  for (int i = 0; i < static_cast<int>(x_right.size()); ++i) {
    ProjPoint pt(x_right[i], y_right[i], z_right[i], 0.0, 0.0, 0.0);
    ls_right.push_back(pt);
  }
}
}  // namespace tam::mapping
