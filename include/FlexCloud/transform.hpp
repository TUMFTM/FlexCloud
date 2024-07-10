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
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <algorithm>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <string>
#include <vector>

#include "triangulation.hpp"
#include "umeyama.hpp"
#include "utility.hpp"
namespace FlexCloud
{
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
   * @param[in] node                - rclcpp::Node:
   *                                  reference to node
   * @param[in] src                 - std::vector<ProjPoint>:
   *                                  source trajectory
   * @param[in] target              - std::vector<ProjPoint>:
   *                                  target trajectory
   * @param[in] umeyama             - std::shared_ptr<Umeyama>:
   *                                  pointer to Umeyama transformation
   * @param[out]                    - bool:
   *                                  true if function executed
   */
  bool get_umeyama(
    rclcpp::Node & node, const std::vector<ProjPoint> & src, const std::vector<ProjPoint> & target,
    const std::shared_ptr<Umeyama> & umeyama);

  /**
   * @brief select control points automatically or manually
   *
   * @param[in] node                - rclcpp::Node:
   *                                  reference to node
   * @param[in] src                 - std::vector<ProjPoint>:
   *                                  source trajectory
   * @param[in] target              - std::vector<ProjPoint>:
   *                                  target trajectory
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  selected control points
   * @param[out]                    - bool:
   *                                  true if function executed
   */
  bool select_control_points(
    rclcpp::Node & node, const std::vector<ProjPoint> & src, const std::vector<ProjPoint> & target,
    std::vector<ControlPoint> & cps);

  /**
   * @brief calculate Rubber-Sheet transformation from target trajectory and selected control points
   *
   * @param[in] node                - rclcpp::Node:
   *                                  reference to node
   * @param[in] target              - std::vector<ProjPoint>:
   *                                  target trajectory
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  control points
   * @param[in] triag               - std::shared_ptr<Delaunay>:
   *                                  pointer to triangulation
   * @param[out]                    - bool:
   *                                  true if function executed
   */
  bool get_rubber_sheeting(
    rclcpp::Node & node, const std::vector<ProjPoint> & target, std::vector<ControlPoint> & cps,
    const std::shared_ptr<Delaunay> triag);

  /**
   * @brief transform linestring with Umeyama trafo
   *
   * @param[in] ls                  - std::vector<ProjPoint>:
   *                                  input linestring
   * @param[in] ls_trans            - std::vector<ProjPoint>:
   *                                  transformed linestring
   * @param[in] umeyama             - std::shared_ptr<Umeyama>:
   *                                  pointer to Umeyama transformation
   * @param[out]                    - bool:
   *                                  true if function executed
   */
  bool transform_ls_al(
    const std::vector<ProjPoint> & ls, std::vector<ProjPoint> & ls_trans,
    const std::shared_ptr<Umeyama> & umeyama);

  /**
   * @brief transform linestring with Rubber-Sheeting trafo
   *
   * @param[in] ls                  - std::vector<ProjPoint>:
   *                                  input linestring
   * @param[in] ls_trans            - std::vector<ProjPoint>:
   *                                  transformed linestring
   * @param[in] triag               - std::shared_ptr<Delaunay>:
   *                                  pointer to triangulation
   * @param[out]                    - bool:
   *                                  true if function executed
   */
  bool transform_ls_rs(
    const std::vector<ProjPoint> & ls, std::vector<ProjPoint> & ls_trans,
    const std::shared_ptr<Delaunay> & triag);

  /**
   * @brief transform point cloud map with Umeyama and Rubber-Sheeting trafo
   *
   * @param[in] node                - rclcpp::Node:
   *                                  reference to node
   * @param[in] umeyama             - std::shared_ptr<Umeyama>:
   *                                  pointer to Umeyama transformation
   * @param[in] triag               - std::shared_ptr<Delaunay>:
   *                                  pointer to triangulation
   * @param[in] pcm                 - pcl::PointCloud<pcl::PointXYZ>::Ptr:
   *                                  pointer to point cloud map
   * @param[out]                    - bool:
   *                                  true if function executed
   */
  bool transform_pcd(
    rclcpp::Node & node, const std::shared_ptr<Umeyama> & umeyama,
    const std::shared_ptr<Delaunay> & triag, const pcl::PointCloud<pcl::PointXYZI>::Ptr & pcm);

  /**
   * @brief transform point cloud map with Umeyama and Rubber-Sheeting trafo using multi-threading
   *
   * @param[in] node                - rclcpp::Node:
   *                                  reference to node
   * @param[in] umeyama             - std::shared_ptr<Umeyama>:
   *                                  pointer to Umeyama transformation
   * @param[in] triag               - std::shared_ptr<Delaunay>:
   *                                  pointer to triangulation
   * @param[in] pcm                 - pcl::PointCloud<pcl::PointXYZ>::Ptr:
   *                                  pointer to point cloud map
   * @param[in] num_cores           - int:
   *                                  amount of cores to be used
   * @param[out]                    - bool:
   *                                  true if function executed
   */
  bool transform_pcd(
    rclcpp::Node & node, const std::shared_ptr<Umeyama> & umeyama,
    const std::shared_ptr<Delaunay> & triag, pcl::PointCloud<pcl::PointXYZI>::Ptr & pcm,
    const int num_cores);

private:
  // Variables for multi-threading
  std::vector<int> currentProgress;
  std::vector<bool> threadsFinished;

  /**
   * @brief get closest point on a linestring for a given point
   *
   * @param[in] pt                    - ProjPoint:
   *                                    input point to be modified
   * @param[in] ls                    - std::vector<ProjPoint>:
   *                                    linestring to match point to
   */
  void closest_on_ls(ProjPoint & pt, const std::vector<ProjPoint> & ls);

  /**
   * @brief transform sub point cloud map one one thread
   *
   * @param[in] threadNum           - int:
   *                                  number of thread
   * @param[in] umeyama             - std::shared_ptr<Umeyama>:
   *                                  pointer to Umeyama transformation
   * @param[in] triag               - std::shared_ptr<Delaunay>:
   *                                  pointer to triangulation
   * @param[in] cloud_in            - pcl::PointCloud<pcl::PointXYZ>::Ptr:
   *                                  pointer to input point cloud map
   * @param[in] cloud_out           - pcl::PointCloud<pcl::PointXYZ>::Ptr:
   *                                  pointer to output point cloud map
   */
  void transform_sub_pcd(
    const int threadNum, const std::shared_ptr<Umeyama> & umeyama,
    const std::shared_ptr<Delaunay> & triag, const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud_out);

  /**
   * @brief set class variables to preprare threading
   *
   * @param[in] num_threads         - size_t:
   *                                  number of thread
   */
  void prepThreading(size_t num_threads);
};
}  // namespace FlexCloud
