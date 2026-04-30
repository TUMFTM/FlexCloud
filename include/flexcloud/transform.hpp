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
//
#include <math.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "cli/cli_config.hpp"
#include "triangulation.hpp"
#include "umeyama.hpp"
#include "utility.hpp"
namespace flexcloud
{
namespace config
{
struct GeoreferencingConfig;  // defined in cli/cli_config.hpp
}
/**
 * @brief class to perform all transformations
 */
class transform
{
public:
  // Class constructor
  transform() {}
  /**
   * @brief calculate Umeyama transformation from source and target trajectory
   *
   * @param[in] src                 - std::vector<PointStdDevStamped>:
   *                                  source trajectory
   * @param[in] target              - std::vector<PoseStamped>:
   *                                  target trajectory
   * @param[in] umeyama             - std::shared_ptr<Umeyama>:
   *                                  pointer to Umeyama transformation
   * @param[out]                    - bool:
   *                                  true if function executed
   */
  bool get_umeyama(
    const std::vector<PointStdDevStamped> & src, const std::vector<PoseStamped> & target,
    const std::shared_ptr<Umeyama> & umeyama);

  /**
   * @brief select control points automatically or manually
   *
   * @param[in] node                - rclcpp::Node:
   *                                  reference to node
   * @param[in] src                 - std::vector<PointStdDevStamped>:
   *                                  source trajectory
   * @param[in] target              - std::vector<PointStdDevStamped>:
   *                                  target trajectory
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  selected control points
   * @param[out]                    - bool:
   *                                  true if function executed
   */
  bool select_control_points(
    config::GeoreferencingConfig & cfg, const std::vector<PointStdDevStamped> & src,
    const std::vector<PoseStamped> & target, std::vector<ControlPoint> & cps);

  /**
   * @brief calculate Rubber-Sheet transformation from target trajectory and selected control points
   *
   * @param[in] node                - rclcpp::Node:
   *                                  reference to node
   * @param[in] target              - std::vector<PointStdDevStamped>:
   *                                  target trajectory
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  control points
   * @param[in] triag               - std::shared_ptr<Delaunay>:
   *                                  pointer to triangulation
   * @param[out]                    - bool:
   *                                  true if function executed
   */
  bool get_rubber_sheeting(
    config::GeoreferencingConfig & cfg, const std::vector<PoseStamped> & target,
    std::vector<ControlPoint> & cps, const std::shared_ptr<Delaunay> & triag);

  /**
   * @brief transform linestring with Umeyama trafo
   *
   * @param[in] ls                  - std::vector<PoseStamped>:
   *                                  input linestring
   * @param[in] ls_trans            - std::vector<PoseStamped>:
   *                                  transformed linestring
   * @param[in] umeyama             - std::shared_ptr<Umeyama>:
   *                                  pointer to Umeyama transformation
   * @param[out]                    - bool:
   *                                  true if function executed
   */
  bool transform_ls_al(
    const std::vector<PoseStamped> & ls, std::vector<PoseStamped> & ls_trans,
    const std::shared_ptr<Umeyama> & umeyama);

  /**
   * @brief transform linestring with Rubber-Sheeting trafo
   *
   * @param[in] ls                  - std::vector<PoseStamped>:
   *                                  input linestring
   * @param[in] ls_trans            - std::vector<PoseStamped>:
   *                                  transformed linestring
   * @param[in] triag               - std::shared_ptr<Delaunay>:
   *                                  pointer to triangulation
   * @param[out]                    - bool:
   *                                  true if function executed
   */
  bool transform_ls_rs(
    const std::vector<PoseStamped> & ls, std::vector<PoseStamped> & ls_trans,
    const std::shared_ptr<Delaunay> & triag);

  /**
   * @brief transform a point cloud map with Umeyama + Rubber-Sheeting using all
   *        hardware threads. Operates on `pcl::PCLPointCloud2`, transforming
   *        only the x/y/z fields and leaving every other field untouched.
   *        Points outside the rubber-sheet
   *        triangulation are dropped.
   *
   * @param[in]    umeyama    Umeyama transformation
   * @param[in]    triag      rubber-sheet triangulation
   * @param[inout] pcm        point cloud map; modified in place (and replaced
   *                          when outliers are removed)
   * @return true if executed
   */
  bool transform_pcd(
    const std::shared_ptr<Umeyama> & umeyama, const std::shared_ptr<Delaunay> & triag,
    pcl::PCLPointCloud2::Ptr & pcm);
};
}  // namespace flexcloud
