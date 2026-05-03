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

#include "visualization.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <pcl/conversions.h>
#include <pcl/filters/impl/approximate_voxel_grid.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <string>
#include <vector>
namespace flexcloud
{
rerun::Color jet(double t)
{
  t = std::clamp(t, 0.0, 1.0);
  const double r = std::clamp(1.5 - std::abs(4.0 * t - 3.0), 0.0, 1.0);
  const double g = std::clamp(1.5 - std::abs(4.0 * t - 2.0), 0.0, 1.0);
  const double b = std::clamp(1.5 - std::abs(4.0 * t - 1.0), 0.0, 1.0);
  return rerun::Color(
    static_cast<uint8_t>(r * 255.0), static_cast<uint8_t>(g * 255.0),
    static_cast<uint8_t>(b * 255.0));
}
/**
 * @brief visualize odom frames in rerun
 *
 * @param[in] pose                 - std::vector<PoseStamped>:
 *                                  odom points
 * @param[in] stream              - rerun::RecordingStream:
 *                                  stream to add linestring to
 * @param[in] color               - std_msgs::msg::ColorRGBA:
 *                                  color of array
 * @param[in] name                - std::string:
 *                                  namespace of linestring
 */
void visualization::pose2rerun(
  const std::vector<PoseStamped> & poses, rerun::RecordingStream & stream, const std::string color,
  const std::string name)
{
  // Convert to rerun
  TUMcolor col(color);
  std::vector<rerun::Position3D> positions{};
  positions.reserve(poses.size());
  for (const auto & p : poses) {
    positions.push_back(
      rerun::Position3D(
        p.pose.pose.translation().x(), p.pose.pose.translation().y(),
        p.pose.pose.translation().z()));
  }
  stream.log(name, rerun::Points3D(positions).with_colors(rerun::Color(col.r, col.g, col.b)));
}
/**
 * @brief visualize pos frames in rerun
 *
 * @param[in] pos                 - std::vector<PointStdDevStamped>:
 *                                  pos points
 * @param[in] stream              - rerun::RecordingStream:
 *                                  stream to add linestring to
 * @param[in] color               - std_msgs::msg::ColorRGBA:
 *                                  color of array
 * @param[in] name                - std::string:
 *                                  namespace of linestring
 */
void visualization::pos2rerun(
  std::vector<PointStdDevStamped> & pos, rerun::RecordingStream & stream, const std::string color,
  const std::string name)
{
  // Convert to rerun
  TUMcolor col(color);
  std::vector<rerun::Position3D> positions{};
  positions.reserve(pos.size());
  for (const auto & p : pos) {
    positions.push_back(rerun::Position3D(p.point.pos.x(), p.point.pos.y(), p.point.pos.z()));
  }
  stream.log(name, rerun::Points3D(positions).with_colors(rerun::Color(col.r, col.g, col.b)));
}
/**
 * @brief visualize linestring in rerun
 *
 * @param[in] ls                  - std::vector<PointStdDevStamped>:
 *                                  controlpoints
 * @param[in] stream              - rerun::RecordingStream:
 *                                  stream to add linestring to
 * @param[in] color               - std_msgs::msg::ColorRGBA:
 *                                  color of array
 * @param[in] name                - std::string:
 *                                  namespace of linestring
 */
void visualization::linestring2rerun(
  const std::vector<PointStdDevStamped> & ls, rerun::RecordingStream & stream,
  const std::string color, const std::string name)
{
  TUMcolor col(color);
  std::vector<rerun::Position3D> positions{};
  positions.reserve(ls.size());
  for (const auto & p : ls) {
    positions.push_back(rerun::Position3D(p.point.pos.x(), p.point.pos.y(), p.point.pos.z()));
  }
  std::vector<rerun::LineStrip3D> lines;
  std::vector<rerun::components::Text> labels;
  for (size_t i = 0; i < positions.size() - 1; ++i) {
    lines.emplace_back(rerun::LineStrip3D({positions[i].xyz, positions[i + 1].xyz}));
    labels.emplace_back(rerun::components::Text(std::to_string(i)));
  }
  stream.log(
    name.c_str(),
    rerun::LineStrips3D(lines).with_colors(rerun::Color(col.r, col.g, col.b)).with_labels(labels));
}
/**
 * @brief visualize linestring in rerun
 *
 * @param[in] ls                  - std::vector<PoseStamped>:
 *                                  controlpoints
 * @param[in] stream              - rerun::RecordingStream:
 *                                  stream to add linestring to
 * @param[in] color               - std_msgs::msg::ColorRGBA:
 *                                  color of array
 * @param[in] name                - std::string:
 *                                  namespace of linestring
 */
void visualization::linestring2rerun(
  const std::vector<PoseStamped> & ls, rerun::RecordingStream & stream, const std::string color,
  const std::string name)
{
  TUMcolor col(color);
  std::vector<rerun::Position3D> positions{};
  positions.reserve(ls.size());
  for (const auto & p : ls) {
    positions.push_back(
      rerun::Position3D(
        p.pose.pose.translation().x(), p.pose.pose.translation().y(),
        p.pose.pose.translation().z()));
  }
  std::vector<rerun::LineStrip3D> lines;
  std::vector<rerun::components::Text> labels;
  for (size_t i = 0; i < positions.size() - 1; ++i) {
    lines.emplace_back(rerun::LineStrip3D({positions[i].xyz, positions[i + 1].xyz}));
    labels.emplace_back(rerun::components::Text(std::to_string(i)));
  }
  stream.log(
    name.c_str(),
    rerun::LineStrips3D(lines).with_colors(rerun::Color(col.r, col.g, col.b)).with_labels(labels));
}
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
void visualization::rs2rerun(
  const std::vector<ControlPoint> & cps, std::shared_ptr<Delaunay> & triag,
  rerun::RecordingStream & stream, const std::string color)
{
  // Control points
  TUMcolor col(color);
  std::vector<rerun::Position3D> positions_cps{};
  positions_cps.reserve(cps.size());
  for (const auto & cp : cps) {
    const Eigen::Vector3d source = cp.source;
    const Eigen::Vector3d target = cp.target;
    positions_cps.push_back(rerun::Position3D(source.x(), source.y(), source.z()));
    positions_cps.push_back(rerun::Position3D(target.x(), target.y(), target.z()));
  }
  stream.log(
    "control_points", rerun::Points3D(positions_cps)
                        .with_colors(rerun::Color(col.r, col.g, col.b))
                        .with_radii({0.5f}));

  // Triangulation
  TUMcolor col_edges("WEBBlueLight");
  std::vector<std::vector<Eigen::Vector3d>> edges = triag->getEdges();
  std::vector<rerun::Position3D> positions_edges{};
  positions_edges.reserve(edges.size() * 2);
  for (const auto & edge : edges) {
    for (const auto & pt : edge) {
      positions_edges.push_back(rerun::Position3D(pt.x(), pt.y(), pt.z()));
    }
  }
  std::vector<rerun::LineStrip3D> lines;
  for (size_t i = 0; i < positions_edges.size() - 1; ++i) {
    lines.emplace_back(rerun::LineStrip3D({positions_edges[i].xyz, positions_edges[i + 1].xyz}));
  }
  stream.log(
    "tetrahedra", rerun::LineStrips3D(lines)
                    .with_colors(rerun::Color(col_edges.r, col_edges.g, col_edges.b))
                    .with_radii({0.3f}));
}
void visualization::linestring2rerun_colored(
  const std::vector<PoseStamped> & ls, const std::vector<double> & values,
  rerun::RecordingStream & stream, const std::string name)
{
  if (ls.size() < 2 || values.empty()) return;

  const double vmin = *std::min_element(values.begin(), values.end());
  const double vmax = *std::max_element(values.begin(), values.end());
  const double span = (vmax - vmin) > 1e-12 ? (vmax - vmin) : 1.0;

  std::vector<rerun::Position3D> positions;
  positions.reserve(ls.size());
  for (const auto & p : ls) {
    positions.push_back(
      rerun::Position3D(
        p.pose.pose.translation().x(), p.pose.pose.translation().y(),
        p.pose.pose.translation().z()));
  }

  std::vector<rerun::LineStrip3D> lines;
  std::vector<rerun::Color> colors;
  std::vector<rerun::components::Text> labels;
  lines.reserve(positions.size() - 1);
  colors.reserve(positions.size() - 1);
  labels.reserve(positions.size() - 1);
  for (size_t i = 0; i < positions.size() - 1; ++i) {
    lines.emplace_back(rerun::LineStrip3D({positions[i].xyz, positions[i + 1].xyz}));
    const double v = (i < values.size()) ? values[i] : values.back();
    colors.emplace_back(jet((v - vmin) / span));
    labels.emplace_back(rerun::components::Text(std::to_string(v)));
  }
  stream.log(
    name.c_str(), rerun::LineStrips3D(lines).with_colors(colors).with_labels(labels));
}
/**
 * @brief visualize point cloud map in rerun
 *
 * @param[in] pcd_map             - pcl::PointCloud<pcl::PointXYZ>::Ptr:
 *                                  pcd map
 * @param[in] stream              - rerun::RecordingStream:
 *                                  stream to add map to
 */
void visualization::pc_map2rerun(
  const pcl::PCLPointCloud2::Ptr & pcd_map, rerun::RecordingStream & stream)
{
  // Convert to plain PointXYZ for voxel filtering and rerun positions — the
  // arbitrary fields of the input cloud are irrelevant for visualization.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*pcd_map, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> grid;
  grid.setInputCloud(cloud);
  grid.setLeafSize(3.0, 3.0, 3.0);
  grid.filter(*cloud_filtered);

  TUMcolor col("White");
  std::vector<rerun::Position3D> positions;
  positions.reserve(cloud_filtered->size());
  for (const auto & p : cloud_filtered->points) {
    positions.push_back(rerun::Position3D(p.x, p.y, p.z));
  }
  stream.log("pcd_map", rerun::Points3D(positions).with_colors(rerun::Color(col.r, col.g, col.b)));
}
}  // namespace flexcloud
