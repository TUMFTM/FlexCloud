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

#include <algorithm>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "triangulation.hpp"
#include "utility.hpp"
namespace FlexCloud
{
/**
 * @brief initialize empty marker as visualization_msgs
 *
 * @param[in] marker              - visualization_msgs::msg::Marker:
 *                                  marker of linestring
 * @param[in] frame_id            - std::string:
 *                                  frame id of marker
 * @param[in] ns                  - std::string:
 *                                  namespace of marker
 * @param[in] c                   - std_msgs::msg::ColorRGBA:
 *                                  color of marker
 */
inline void init_linestring_marker(
  visualization_msgs::msg::Marker * marker, const std::string & frame_id, const std::string & ns,
  const std_msgs::msg::ColorRGBA & c)
{
  if (marker == nullptr) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("visualization"), __FUNCTION__ << ": marker is null pointer!");
    return;
  }

  marker->header.frame_id = frame_id;
  marker->header.stamp = rclcpp::Time();
  marker->frame_locked = false;
  marker->ns = ns;
  marker->action = visualization_msgs::msg::Marker::ADD;
  marker->type = visualization_msgs::msg::Marker::LINE_STRIP;

  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = 0.0;
  marker->pose.orientation.w = 1.0;
  marker->scale.x = 1.0;
  marker->scale.y = 1.0;
  marker->scale.z = 1.0;
  marker->color = c;
}
/**
 * @brief add colored vector of points to marker
 *
 * @param[in] marker              - visualization_msgs::msg::Marker:
 *                                  marker of linestring
 * @param[in] ls                  - std::vector<ProjPoint>:
 *                                  vector of points
 * @param[in] c                   - std_msgs::msg::ColorRGBA:
 *                                  color of marker
 */
inline void push_linestring_marker(
  visualization_msgs::msg::Marker * marker, const std::vector<ProjPoint> & ls,
  const std_msgs::msg::ColorRGBA & c)
{
  if (marker == nullptr) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("visualization"), __FUNCTION__ << ": marker is null pointer!");
    return;
  }

  // fill out lane line
  if (ls.size() < 2) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("visualization"), __FUNCTION__ << ": marker line size is 1 or 0!");
    return;
  }
  for (const auto & point : ls) {
    geometry_msgs::msg::Point p;
    p.x = point.pos_(0);
    p.y = point.pos_(1);
    p.z = point.pos_(2);
    marker->points.push_back(p);
    marker->colors.push_back(c);
  }
}
/**
 * @brief insert one marker array into another
 *
 * @param[in] arr_src             - visualization_msgs::msg::MarkerArray:
 *                                  source marker array
 * @param[in] arr_in              - visualization_msgs::msg::MarkerArray:
 *                                  array to insert
 */
inline void insert_marker_array(
  visualization_msgs::msg::MarkerArray * arr_src,
  const visualization_msgs::msg::MarkerArray & arr_in)
{
  arr_src->markers.insert(arr_src->markers.end(), arr_in.markers.begin(), arr_in.markers.end());
}
/**
 * @brief set color from name and intensity
 *
 * @param[in] color               - std_msgs::msg::ColorRGBA:
 *                                  resulting color message
 * @param[in] name                - std::string:
 *                                  name of color
 * @param[in] a                   - double:
 *                                  intensity of color
 */
inline void set_color(std_msgs::msg::ColorRGBA * color, const std::string & name, double a)
{
  TUMcolor cl(name);
  color->r = cl.r;
  color->g = cl.g;
  color->b = cl.b;
  color->a = a;
}
/**
 * @brief convert linestring to marker array
 *
 * @param[in] ls                  - std::vector<ProjPoint>:
 *                                  linestring
 * @param[in] ns                  - std::string:
 *                                  namespace of markerarray
 * @param[in] c                   - std_msgs::msg::ColorRGBA:
 *                                  color of array
 * @param[out]                    - visualization_msgs::msg::MarkerArray:
 *                                  marker array
 */
inline visualization_msgs::msg::MarkerArray linestring2marker(
  const std::vector<ProjPoint> & ls, const std::string & ns, const std_msgs::msg::ColorRGBA & c)
{
  visualization_msgs::msg::MarkerArray marker_array;
  if (ls.empty()) {
    return marker_array;
  }
  visualization_msgs::msg::Marker marker;
  init_linestring_marker(&marker, "map", ns, c);

  push_linestring_marker(&marker, ls, c);
  marker_array.markers.push_back(marker);
  return marker_array;
}
/**
 * @brief convert vector of linestrings to marker array
 *
 * @param[in] linestrings         - std::vector<std::vector<ProjPoint>>:
 *                                  linestrings
 * @param[in] ns                  - std::string:
 *                                  namespace of markerarray
 * @param[in] c                   - std_msgs::msg::ColorRGBA:
 *                                  color of array
 * @param[out]                    - visualization_msgs::msg::MarkerArray:
 *                                  marker array
 */
inline visualization_msgs::msg::MarkerArray linestrings2marker(
  const std::vector<std::vector<ProjPoint>> & linestrings, const std::string & ns,
  const std_msgs::msg::ColorRGBA & c)
{
  visualization_msgs::msg::MarkerArray marker_array;
  if (linestrings.empty()) {
    return marker_array;
  }
  int i = 1;
  for (const std::vector<ProjPoint> & ls : linestrings) {
    std::string ns_ = ns + "_" + std::to_string(i);
    insert_marker_array(&marker_array, linestring2marker(ls, ns_, c));
    ++i;
  }
  return marker_array;
}
/**
 * @brief convert indices of linestring elements to marker array
 *
 * @param[in] ls                  - std::vector<ProjPoint>:
 *                                  linestring
 * @param[in] ns                  - std::string:
 *                                  namespace of markerarray
 * @param[in] c                   - std_msgs::msg::ColorRGBA:
 *                                  color of array
 * @param[out]                    - visualization_msgs::msg::MarkerArray:
 *                                  marker array
 */
inline visualization_msgs::msg::MarkerArray ids2marker(
  const std::vector<ProjPoint> & ls, const std::string & ns, const std_msgs::msg::ColorRGBA & c)
{
  visualization_msgs::msg::MarkerArray id_marker_array;

  int i = 0;
  for (const auto & pt : ls) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Time();
    marker.ns = ns;
    marker.id = static_cast<int32_t>(i);
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = pt.pos_(0);
    marker.pose.position.y = pt.pos_(1) + 1;
    marker.pose.position.z = pt.pos_(2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color = c;
    marker.scale.z = 1.0;
    marker.frame_locked = false;
    marker.text = std::to_string(i);

    id_marker_array.markers.push_back(marker);
    ++i;
  }

  return id_marker_array;
}
/**
 * @brief convert vector of controlpoints to marker array
 *
 * @param[in] cps                 - std::vector<ControlPoint>>:
 *                                  controlpoints
 * @param[in] ns                  - std::string:
 *                                  namespace of markerarray
 * @param[in] c                   - std_msgs::msg::ColorRGBA:
 *                                  color of array
 * @param[out]                    - visualization_msgs::msg::MarkerArray:
 *                                  marker array
 */
inline visualization_msgs::msg::MarkerArray controlpoints2marker(
  const std::vector<ControlPoint> & cps, const std::string color, const std::string ns)
{
  visualization_msgs::msg::MarkerArray msg;
  std_msgs::msg::ColorRGBA col;
  set_color(&col, color, 0.999);

  int i = 1;
  for (auto & cpt : cps) {
    std::string ns_ind = ns + "_" + std::to_string(i);
    std::vector<ProjPoint> ls{
      ProjPoint(
        cpt.get_source_point()(0), cpt.get_source_point()(1), cpt.get_source_point()(2), 0.0, 0.0,
        0.0),
      ProjPoint(
        cpt.get_target_point()(0), cpt.get_target_point()(1), cpt.get_target_point()(2), 0.0, 0.0,
        0.0)};
    insert_marker_array(&msg, linestring2marker(ls, ns_ind, col));
    i += 1;
  }
  return msg;
}
/**
 * @brief convert triangulation edges to marker array
 *
 * @param[in] triag               - std::shared_ptr<Delaunay>:
 *                                  pointer to triangulation
 * @param[in] ns                  - std::string:
 *                                  namespace of markerarray
 * @param[in] c                   - std_msgs::msg::ColorRGBA:
 *                                  color of array
 * @param[out]                    - visualization_msgs::msg::MarkerArray:
 *                                  marker array
 */
inline visualization_msgs::msg::MarkerArray triag2marker(
  std::shared_ptr<Delaunay> & triag, const std::string color, const std::string ns)
{
  visualization_msgs::msg::MarkerArray msg;
  std_msgs::msg::ColorRGBA col;
  set_color(&col, color, 0.999);
  std::vector<std::vector<ProjPoint>> edges = triag->getEdges();

  insert_marker_array(&msg, linestrings2marker(edges, ns, col));
  return msg;
}
}  // namespace FlexCloud
