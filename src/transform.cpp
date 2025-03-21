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

#include "transform.hpp"
namespace flexcloud
{
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
bool transform::get_umeyama(
  rclcpp::Node & node, const std::vector<ProjPoint> & src, const std::vector<ProjPoint> & target,
  const std::shared_ptr<Umeyama> & umeyama)
{
  (void)node;
  // Convert vector to Eigen format
  std::vector<Eigen::Vector3d> sr, tar;
  sr.clear();
  tar.clear();
  for (const auto & pt : src) {
    sr.push_back(pt.pos_);
  }
  for (const auto & pt : target) {
    tar.push_back(pt.pos_);
  }

  // Insert into Umeyama class and calculate transformation matrix
  umeyama->insertSource(sr);
  umeyama->insertTarget(tar);
  umeyama->calcTransform();
  return true;
}
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
bool transform::select_control_points(
  rclcpp::Node & node, const std::vector<ProjPoint> & src, const std::vector<ProjPoint> & target,
  std::vector<ControlPoint> & cps)
{
  // Get node name
  const std::string node_name = node.get_parameter("node_name").as_string();
  cps.clear();

  // Get amount of control points from user input
  const int num_control_points = node.get_parameter("rs_num_controlPoints").as_int();
  // Get threshold for standard deviation of GPS points
  const double stddev_threshold = node.get_parameter("stddev_threshold").as_double();

  std::vector<Eigen::Vector3d> cp_inter;
  int cp_count = 0;

  std::vector<int64_t> exclude_ind = node.get_parameter("exclude_ind").as_integer_array();
  // Manually shift GPS point from config file
  std::vector<int64_t> shift_ind = node.get_parameter("shift_ind").as_integer_array();
  std::vector<double> shift_ind_dist = node.get_parameter("shift_ind_dist").as_double_array();
  if (shift_ind.size() != shift_ind_dist.size()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(node_name), "Sizes of shift_ind and shift_ind_dist do not match!");
  }
  // Code for when use_gpControlPoints is true
  int gps_distance = static_cast<int>(target.size()) / num_control_points;
  int idx = 0;

  std::cout << "\033[33m~~~~~> LiDAR got " << target.size() << " poses!\033[0m" << std::endl;
  std::cout << "\033[33m~~~~~> Every " << gps_distance
            << " st/nd/rd/th vertex is selected as control point\033[0m" << std::endl;

  while (idx < static_cast<int>(target.size())) {
    // Chech if index is within manually excluded indices
    bool use_ind = true;
    if (!exclude_ind.empty() || static_cast<int>(exclude_ind.size()) % 2 != 0) {
      for (int i = 0; i < static_cast<int>(exclude_ind.size()) / 2; ++i) {
        if (idx >= exclude_ind[2 * i] && idx <= exclude_ind[2 * i + 1]) {
          use_ind = false;
        }
      }
    }
    /* Point in GPS data -check if standard deviations are within threshold, otherwise skip
     * point*/
    if (
      sqrt(pow(src[idx].stddev_(0), 2) + pow(src[idx].stddev_(1), 2)) <= stddev_threshold &&
      use_ind) {
      // Shift reference point if in config file
      if (std::find(shift_ind.begin(), shift_ind.end(), idx) != shift_ind.end()) {
        const int dist_ind = std::find(shift_ind.begin(), shift_ind.end(), idx) - shift_ind.begin();
        std::cout << "\033[33m~~~~~> Shift reference point at index: " << idx << "with distance "
                  << shift_ind_dist[dist_ind] << "\033[0m" << std::endl;
        // Create vincinity = pair of preceding and following point on linestring
        std::vector<Eigen::Vector3d> vincinity{};
        Eigen::Vector3d current = src[idx].pos_;
        if (idx == 0) {
          Eigen::Vector3d forward = src[idx + 1].pos_;
          vincinity =
            std::vector<Eigen::Vector3d>{Eigen::Vector3d(0.0, 0.0, 0.0), forward - current};
        } else if (idx + 1 == static_cast<int>(src.size())) {
          Eigen::Vector3d backward = src[idx - 1].pos_;
          vincinity =
            std::vector<Eigen::Vector3d>{current - backward, Eigen::Vector3d(0.0, 0.0, 0.0)};
        } else {
          Eigen::Vector3d backward = src[idx - 1].pos_;
          Eigen::Vector3d forward = src[idx + 1].pos_;
          vincinity = std::vector<Eigen::Vector3d>{current - backward, forward - current};
        }
        // shift point laterally by specified distance
        Eigen::Vector2d perpendicular;
        double realOffset = shift_ind_dist[dist_ind];
        const auto epsilon{1.e-5};
        if (idx == 0) {
          perpendicular = Eigen::Vector2d(vincinity.back()(0), vincinity.back()(1));
        } else if (idx + 1 == static_cast<int>(src.size())) {
          perpendicular = Eigen::Vector2d(vincinity.front()(0), vincinity.front()(1));
        } else {
          perpendicular = Eigen::Vector2d(vincinity.back()(0), vincinity.back()(1)).normalized() +
                          Eigen::Vector2d(vincinity.front()(0), vincinity.front()(1)).normalized();
          auto minussin2 = perpendicular.norm() / 2;
          realOffset = (minussin2 > epsilon) ? shift_ind_dist[dist_ind] / minussin2 : 0;
        }

        Eigen::Vector2d direction(-perpendicular(1), perpendicular(0));
        Eigen::Vector2d pt_2d_shift =
          Eigen::Vector2d(src[idx].pos_(0), src[idx].pos_(1)) + direction.normalized() * realOffset;
        // Create control point
        cp_inter.push_back(Eigen::Vector3d(pt_2d_shift(0), pt_2d_shift(1), src[idx].pos_(2)));
      } else {
        // Set unmodified reference point
        cp_inter.push_back(Eigen::Vector3d(src[idx].pos_(0), src[idx].pos_(1), src[idx].pos_(2)));
      }

      // Point in LiDAR data
      Eigen::Vector3d pt_pcd(target[idx].pos_(0), target[idx].pos_(1), target[idx].pos_(2));
      cp_inter.push_back(pt_pcd);

      ControlPoint P(
        cp_inter[0](0), cp_inter[0](1), cp_inter[0](2), cp_inter[1](0), cp_inter[1](1),
        cp_inter[1](2));
      cps.push_back(P);
      ++cp_count;

      cp_inter.clear();
    } else {
      std::cout << "\033[31m!! Skipped control point !!\033[0m" << std::endl;
    }
    idx += gps_distance;
  }
  std::cout << "\033[1;32m~~~~~~~~~~> Set " << cp_count << " controlpoints!\033[0m" << std::endl;
  return true;
}
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
bool transform::get_rubber_sheeting(
  rclcpp::Node & node, const std::vector<ProjPoint> & target, std::vector<ControlPoint> & cps,
  const std::shared_ptr<Delaunay> triag)
{
  // Calculate enclosing rectangle or cube
  const std::vector<double> size = node.get_parameter("square_size").as_double_array();
  triag->enclosingControlPoints(size, target, cps);
  // Insertion of control points into triangulation structure
  for (unsigned i = 0; i < cps.size(); i++) {
    triag->insertPoint(cps[i].get_target_point());
  }
  triag->mapControlPoints(cps);
  triag->calcTransformations();
  return true;
}
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
bool transform::transform_ls_al(
  const std::vector<ProjPoint> & ls, std::vector<ProjPoint> & ls_trans,
  const std::shared_ptr<Umeyama> & umeyama)
{
  ls_trans.clear();
  // Transform geometry
  for (const auto & pt : ls) {
    Eigen::Vector3d point = pt.pos_;
    umeyama->transformPoint(point);
    ProjPoint pt_t(point(0), point(1), point(2), pt.stddev_(0), pt.stddev_(1), pt.stddev_(2));
    ls_trans.push_back(pt_t);
  }
  return true;
}
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
bool transform::transform_ls_rs(
  const std::vector<ProjPoint> & ls, std::vector<ProjPoint> & ls_trans,
  const std::shared_ptr<Delaunay> & triag)
{
  // Initialize output linestring to be filled -> non-const
  ls_trans.clear();
  // Transform geometry
  for (const auto & pt : ls) {
    // Find area the point is in
    Eigen::Vector3d pt_ = pt.pos_;
    triag->transformPoint(pt_);
    ProjPoint pt_t(pt_(0), pt_(1), pt_(2), pt.stddev_(0), pt.stddev_(1), pt.stddev_(2));
    ls_trans.push_back(pt_t);
  }
  return true;
}
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
bool transform::transform_pcd(
  rclcpp::Node & node, const std::shared_ptr<Umeyama> & umeyama,
  const std::shared_ptr<Delaunay> & triag, const pcl::PointCloud<pcl::PointXYZI>::Ptr & pcm)
{
  (void)node;
  // Check if the input pointer is valid
  if (!pcm) {
    return false;
  }

  // Transform points and write into output cloud
  pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
  int ind_pt = 0;
  for (const auto & point : *pcm) {
    // Align point with alignment transformation matrix
    Eigen::Vector3d pt_al(point.x, point.y, point.z);
    umeyama->transformPoint(pt_al);
    Eigen::Vector3d pt_rs(pt_al(0), pt_al(1), pt_al(2));
    triag->transformPoint(pt_rs);

    if (pt_rs.norm() < 1.0e-3) {
      // Point outside of triangulation
      outliers->indices.push_back(ind_pt);
    } else {
      pcm->points[ind_pt].x = pt_rs(0);
      pcm->points[ind_pt].y = pt_rs(1);
      pcm->points[ind_pt].z = pt_rs(2);
    }
    ++ind_pt;
  }
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(pcm);
  extract.setIndices(outliers);
  extract.setNegative(true);
  extract.filter(*pcm);
  return true;
}
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
bool transform::transform_pcd(
  rclcpp::Node & node, const std::shared_ptr<Umeyama> & umeyama,
  const std::shared_ptr<Delaunay> & triag, pcl::PointCloud<pcl::PointXYZI>::Ptr & pcm,
  const int num_cores)
{
  (void)node;
  // Let's run threading
  int num_max_cores = std::thread::hardware_concurrency();
  size_t num_threads = 0;
  // Check valid number for no. threads
  if (num_cores < num_max_cores) {
    num_threads = num_cores;
  } else {
    num_threads = static_cast<int>(std::floor(num_max_cores / 1.25));
  }
  size_t numPoints = pcm->size();
  size_t quotient = numPoints / num_threads;
  size_t remainder = numPoints % num_threads;
  this->prepThreading(num_threads);

  // Create number of sub pointclouds for threading
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_in_array;
  for (size_t i = 0; i < num_threads; ++i) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_in_array.push_back(cloud_in);
  }

  // Create specific indices for all threads
  std::vector<std::vector<int>> indices;
  for (size_t i = 0; i < num_threads; ++i) {
    std::vector<int> row;
    if (i == 0) {
      for (size_t x = 0; x <= (quotient - 1); ++x) {
        row.push_back(x);
      }
    } else if (i > 0 && i < (num_threads - 1)) {
      for (size_t x = (i * quotient); x <= ((i + 1) * quotient - 1); ++x) {
        row.push_back(x);
      }
    } else if (i == (num_threads - 1)) {
      for (size_t x = (i * quotient); x <= ((i + 1) * quotient + remainder - 1); ++x) {
        row.push_back(x);
      }
    }
    indices.push_back(row);
  }

  // Copy corresponding parts of cloud to smaller pointclouds
  for (size_t i = 0; i < num_threads; ++i) {
    pcl::copyPointCloud(*pcm, indices[i], *cloud_in_array[i]);
  }
  // Configure every pcd
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_out_array;
  for (size_t i = 0; i < num_threads; ++i) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_out->is_dense = false;
    cloud_out->height = 1;

    if (i < (num_threads - 1)) {
      cloud_out->width = quotient;
    } else if (i == (num_threads - 1)) {
      cloud_out->width = quotient + remainder;
    }

    cloud_out->points.resize(cloud_out->width * cloud_out->height);
    cloud_out_array.push_back(cloud_out);
  }

  // Add to vector and start threads
  std::vector<std::thread> threads;
  for (size_t i = 0; i < num_threads; ++i) {
    threads.emplace_back(
      &transform::transform_sub_pcd, this, i, umeyama, triag, cloud_in_array[i],
      cloud_out_array[i]);
  }

  std::cout << "\033[1;36mStart pcd transformation (" << num_threads << " threads started)\033[0m"
            << std::endl;

  // Wait for all threads to finish cleanly
  while (true) {
    // Check if all threads finished
    bool finished = true;
    for (size_t i = 0; i < num_threads; ++i) {
      if (!this->threadsFinished[i]) {
        finished = false;
      }
    }

    // Sum current progress
    int processedPoints = 0;
    for (size_t i = 0; i < num_threads; ++i) {
      processedPoints += this->currentProgress[i];
    }

    float progress = static_cast<float>(processedPoints) / static_cast<float>(numPoints);
    int bar_width = 50;
    static int progress_counter = 0;

    if (progress >= (progress_counter / 10.0)) {
      progress_counter++;

      std::cout << "[";
      int pos = bar_width * progress;

      for (int i = 0; i < bar_width; ++i) {
        if (i < pos)
          std::cout << "=";
        else if (i == pos)
          std::cout << ">";
        else
          std::cout << " ";
      }
      std::cout << "]" << int(progress * 100.0) << " %\r";
      std::cout << std::endl;
    }

    // Break if done
    if (finished) {
      std::cout << "\033[1;36mTransformation finished\033[0m" << std::endl;
      break;
    }
  }

  // Gather all threads just for sure
  for (auto & thread : threads) {
    thread.join();
  }

  // Remove points from input cloud and write transformed points clouds
  pcm->clear();

  for (size_t i = 0; i < num_threads; ++i) {
    *pcm += *cloud_out_array[i];
  }
  return true;
}
/**
 * @brief get closest point on a linestring for a given point
 *
 * @param[in] pt                    - ProjPoint:
 *                                    input point to be modified
 * @param[in] ls                    - std::vector<ProjPoint>:
 *                                    linestring to match point to
 */
void transform::closest_on_ls(ProjPoint & pt, const std::vector<ProjPoint> & ls)
{
  // Initialize
  std::vector<double> dist;
  for (const auto & pt_ : ls) {
    double d = (pt.pos_ - pt_.pos_).norm();
    dist.push_back(d);
  }
  const int ind = std::distance(dist.begin(), std::min_element(dist.begin(), dist.end()));
  pt.pos_(0) = ls[ind].pos_(0);
  pt.pos_(1) = ls[ind].pos_(1);
  pt.pos_(2) = ls[ind].pos_(2);
}
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
void transform::transform_sub_pcd(
  const int threadNum, const std::shared_ptr<Umeyama> & umeyama,
  const std::shared_ptr<Delaunay> & triag, const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud_in,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud_out)
{
  // Transformation begin
  pcl::PointIndices::Ptr outliers(new pcl::PointIndices());
  int ind_pt = 0;
  for (const auto & point : *cloud_in) {
    // Align point with alignment transformation matrix
    Eigen::Vector3d pt_al(point.x, point.y, point.z);
    umeyama->transformPoint(pt_al);

    // Locate new, aligned point
    Eigen::Vector3d pt_rs(pt_al(0), pt_al(1), pt_al(2));
    triag->transformPoint(pt_rs);
    if (pt_rs.norm() < 1.0e-3) {
      // Point outside of triangulation
      outliers->indices.push_back(ind_pt);
    } else {
      // Point inside triangulation
      cloud_out->points[ind_pt].x = pt_rs(0);
      cloud_out->points[ind_pt].y = pt_rs(1);
      cloud_out->points[ind_pt].z = pt_rs(2);
    }
    // Keep intensity
    cloud_out->points[ind_pt].intensity = point.intensity;
    ++ind_pt;
    this->currentProgress[threadNum] = ind_pt;
  }
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cloud_out);
  extract.setIndices(outliers);
  extract.setNegative(true);
  extract.filter(*cloud_out);

  this->threadsFinished[threadNum] = true;
}
/**
 * @brief set class variables to preprare threading
 *
 * @param[in] num_threads         - size_t:
 *                                  number of thread
 */
void transform::prepThreading(size_t num_threads)
{
  this->threadsFinished.resize(num_threads);
  this->currentProgress.resize(num_threads);
  for (size_t i = 0; i < num_threads; ++i) {
    this->threadsFinished[i] = false;
    this->currentProgress[i] = 0;
  }
}
}  // namespace flexcloud
