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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include <pcl/PCLPointField.h>
#include <pcl/common/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/impl/pcl_base.hpp>

#include "cli/cli_config.hpp"

namespace flexcloud
{
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
bool transform::get_umeyama(
  const std::vector<PointStdDevStamped> & src, const std::vector<PoseStamped> & target,
  const std::shared_ptr<Umeyama> & umeyama)
{
  // Convert vector to Eigen format
  std::vector<Eigen::Vector3d> sr{}, tar{};
  for (const auto & pt : src) {
    sr.push_back(pt.point.pos);
  }
  for (const auto & pt : target) {
    tar.push_back(pt.pose.pose.translation());
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
 * @param[in] cfg                 - config::GeoreferencingConfig:
 *                                 reference to config
 * @param[in] src                 - std::vector<PointStdDev>:
 *                                  source trajectory
 * @param[in] target              - std::vector<PointStdDev>:
 *                                  target trajectory
 * @param[in] cps                 - std::vector<ControlPoint>:
 *                                  selected control points
 * @param[out]                    - bool:
 *                                  true if function executed
 */
bool transform::select_control_points(
  config::GeoreferencingConfig & cfg, const std::vector<PointStdDevStamped> & src,
  const std::vector<PoseStamped> & target, std::vector<ControlPoint> & cps)
{
  cps.clear();
  std::vector<Eigen::Vector3d> cp_inter{};
  int cp_count = 0;

  // Santiy checks
  if (cfg.shift_ind.size() != cfg.shift_ind_dist.size()) {
    std::cout << "Sizes of shift_ind and shift_ind_dist do not match!" << std::endl;
  }
  if (
    cfg.fake_ind.size() != cfg.fake_ind_dist.size() ||
    cfg.fake_ind.size() != cfg.fake_ind_height.size()) {
    std::cout << "Sizes of fake_ind, fake_ind_dist and fake_ind_height do not match!" << std::endl;
  }
  // Set interval for control point selection
  int traj_split = static_cast<int>(target.size()) / cfg.control_points;
  std::cout << "\033[33m~~~~~> LiDAR got " << target.size() << " poses!\033[0m" << std::endl;
  std::cout << "\033[33m~~~~~> Every " << traj_split
            << " st/nd/rd/th vertex is selected as control point\033[0m" << std::endl;

  // Iterate over indices
  for (size_t idx = 0; idx < static_cast<size_t>(target.size()); ++idx) {
    // Check if index is within manually excluded indices
    bool use_ind = true;
    if (!cfg.exclude_ind.empty() || static_cast<int>(cfg.exclude_ind.size()) % 2 != 0) {
      for (int i = 0; i < static_cast<int>(cfg.exclude_ind.size()) / 2; ++i) {
        if (
          idx >= static_cast<size_t>(cfg.exclude_ind[2 * i]) &&
          idx <= static_cast<size_t>(cfg.exclude_ind[2 * i + 1])) {
          use_ind = false;
          break;
        }
      }
    }
    if (!use_ind) {
      continue;
    }
    // clang-format off
    // Check if standard deviations of reference point are within threshold, otherwise skip point
    if (sqrt(pow(src[idx].point.stddev.x(), 2) + pow(src[idx].point.stddev.y(), 2)) > cfg.stddev_threshold) {  // NOLINT
      std::cout << "\033[31m!! Skipped control point due to high stddev !!\033[0m" << std::endl;
      continue;
    }
    // Check if index is used in regular split or faked
    if (idx % traj_split == 0 && std::find(cfg.shift_ind.begin(), cfg.shift_ind.end(), idx) == cfg.shift_ind.end()) {  // NOLINT
      // Point is a regular control point
      // Set unmodified reference point
      cp_inter.push_back(src[idx].point.pos);
      // Set unmodified LiDAR point
      cp_inter.push_back(target[idx].pose.pose.translation());
      ControlPoint P(cp_inter[0], cp_inter[1]);
      cps.push_back(P);
      ++cp_count;
      cp_inter.clear();
    }

    // Now only continue of point is to be shifted or faked
    if (std::find(cfg.shift_ind.begin(), cfg.shift_ind.end(), idx) == cfg.shift_ind.end() &&  // NOLINT
        std::find(cfg.fake_ind.begin(), cfg.fake_ind.end(), idx) == cfg.fake_ind.end()) {  // NOLINT
      continue;
    }
    // Point configured to be shifted or fake => compute vincinity
    double real_offset = 0.0;
    Eigen::Vector2d direction{};
    double distance = 0.0;
    int ind = 0;
    // Compute vincinity
    // Create vincinity = pair of preceding and following point on linestring
    std::vector<Eigen::Vector3d> vincinity{};
    Eigen::Vector3d current = src[idx].point.pos;
    if (idx == 0) {
      Eigen::Vector3d forward = src[idx + 1].point.pos;
      vincinity =
        std::vector<Eigen::Vector3d>{Eigen::Vector3d(0.0, 0.0, 0.0), forward - current};
    } else if (idx + 1 == src.size()) {
      Eigen::Vector3d backward = src[idx - 1].point.pos;
      vincinity =
        std::vector<Eigen::Vector3d>{current - backward, Eigen::Vector3d(0.0, 0.0, 0.0)};
    } else {
      Eigen::Vector3d backward = src[idx - 1].point.pos;
      Eigen::Vector3d forward = src[idx + 1].point.pos;
      vincinity = std::vector<Eigen::Vector3d>{current - backward, forward - current};
    }
    // Check if point is configured to be shifted or faked
    if (std::find(cfg.shift_ind.begin(), cfg.shift_ind.end(), idx) != cfg.shift_ind.end()) {  // NOLINT
      ind = std::find(cfg.shift_ind.begin(), cfg.shift_ind.end(), idx) - cfg.shift_ind.begin();  // NOLINT
      std::cout << "\033[33m~~~~~> Shift reference point at index: " << idx << "with distance "
      << cfg.shift_ind_dist[ind] << "\033[0m" << std::endl;
      distance = cfg.shift_ind_dist[ind];
    } else if (std::find(cfg.fake_ind.begin(), cfg.fake_ind.end(), idx) != cfg.fake_ind.end()) {  // NOLINT
      ind = std::find(cfg.fake_ind.begin(), cfg.fake_ind.end(), idx) - cfg.fake_ind.begin();  // NOLINT
      // Fake point
      std::cout << "\033[33m~~~~~> Fake reference point at index: " << idx << "with distance "
      << cfg.fake_ind_dist[ind] << "\033[0m" << std::endl;
      distance = cfg.fake_ind_dist[ind];
    }
    // Create control point on reference trajectory
    // shift point laterally by specified distance
    Eigen::Vector2d perpendicular;
    real_offset = distance;
    const auto epsilon{1.e-5};
    if (idx == 0) {
      perpendicular = Eigen::Vector2d(vincinity.back()(0), vincinity.back()(1));
    } else if (idx + 1 == src.size()) {
      perpendicular = Eigen::Vector2d(vincinity.front()(0), vincinity.front()(1));
    } else {
      perpendicular = Eigen::Vector2d(vincinity.back()(0), vincinity.back()(1)).normalized() +
                      Eigen::Vector2d(vincinity.front()(0), vincinity.front()(1)).normalized();
      auto minussin2 = perpendicular.norm() / 2;
      real_offset = (minussin2 > epsilon) ? distance / minussin2 : 0;
    }

    direction << -perpendicular(1), perpendicular(0);
    Eigen::Vector2d pt_2d_shift =
      Eigen::Vector2d(src[idx].point.pos.x(), src[idx].point.pos.y()) + direction.normalized() * real_offset;
    // Add shifted reference point
    cp_inter.push_back(Eigen::Vector3d(pt_2d_shift(0), pt_2d_shift(1), src[idx].point.pos.z()));
    // Point in LiDAR data
    // Check if point is configured to be faked
    Eigen::Vector3d pt_pcd{};
    if (std::find(cfg.fake_ind.begin(), cfg.fake_ind.end(), idx) != cfg.fake_ind.end()) {  // NOLINT
      // Fake point
      Eigen::Vector2d pt_pcd_shift = Eigen::Vector2d(target[idx].pose.pose.translation().x(), target[idx].pose.pose.translation().y()) + direction.normalized() * real_offset;  // NOLINT
      pt_pcd << pt_pcd_shift(0), pt_pcd_shift(1), target[idx].pose.pose.translation().z() + cfg.fake_ind_height[ind];  // NOLINT
    } else {
      pt_pcd << target[idx].pose.pose.translation().x(), target[idx].pose.pose.translation().y(), target[idx].pose.pose.translation().z();  // NOLINT
    }
    cp_inter.push_back(pt_pcd);
    ControlPoint P(cp_inter[0], cp_inter[1]);
    cps.push_back(P);
    ++cp_count;
    cp_inter.clear();
    // clang-format on
  }
  std::cout << "\033[1;32m~~~~~~~~~~> Set " << cp_count << " controlpoints!\033[0m" << std::endl;
  return true;
}
/**
 * @brief calculate Rubber-Sheet transformation from target trajectory and selected control points
 *
 * @param[in] cfg                 - config::GeoreferencingConfig:
 *                                  reference to config
 * @param[in] target              - std::vector<PointStdDev>:
 *                                  target trajectory
 * @param[in] cps                 - std::vector<ControlPoint>:
 *                                  control points
 * @param[in] triag               - std::shared_ptr<Delaunay>:
 *                                  pointer to triangulation
 * @param[out]                    - bool:
 *                                  true if function executed
 */
bool transform::get_rubber_sheeting(
  config::GeoreferencingConfig & cfg, const std::vector<PoseStamped> & target,
  std::vector<ControlPoint> & cps, const std::shared_ptr<Delaunay> & triag)
{
  // Convert target trajectory to Eigen format and insert into triangulation
  std::vector<Eigen::Vector3d> target_eigen;
  for (const auto & pose : target) {
    target_eigen.push_back(pose.pose.pose.translation());
  }
  triag->enclosingControlPoints(cfg.square_size, target_eigen, cps);
  // Insertion of control points into triangulation structure
  for (unsigned i = 0; i < cps.size(); i++) {
    triag->insertPoint(cps[i].target);
  }
  triag->mapControlPoints(cps);
  triag->calcTransformations();
  return true;
}
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
bool transform::transform_ls_al(
  const std::vector<PoseStamped> & ls, std::vector<PoseStamped> & ls_trans,
  const std::shared_ptr<Umeyama> & umeyama)
{
  ls_trans.clear();
  // Transform geometry
  for (const auto & pt : ls) {
    PoseStamped ps = pt;
    umeyama->transformPoint(ps.pose.pose);
    ls_trans.emplace_back(ps);
  }
  return true;
}
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
bool transform::transform_ls_rs(
  const std::vector<PoseStamped> & ls, std::vector<PoseStamped> & ls_trans,
  const std::shared_ptr<Delaunay> & triag)
{
  // Initialize output linestring to be filled -> non-const
  ls_trans.clear();
  // Transform geometry
  for (const auto & pose : ls) {
    // Find area the point is in
    Eigen::Isometry3d pose_eigen = pose.pose.pose;
    triag->transformPoint(pose_eigen);
    PoseStamped pose_t = pose;
    pose_t.pose.pose = pose_eigen;
    ls_trans.push_back(pose_t);
  }
  return true;
}
/**
 * @brief Locate the byte offsets of the three coordinate fields and verify
 *        they are all FLOAT32. Throws if not.
 */
struct XYZFieldLayout
{
  std::uint32_t x_off, y_off, z_off, point_step;
};

XYZFieldLayout locate_xyz(const pcl::PCLPointCloud2 & pcm)
{
  XYZFieldLayout out{};
  int x_idx = pcl::getFieldIndex(pcm, "x");
  int y_idx = pcl::getFieldIndex(pcm, "y");
  int z_idx = pcl::getFieldIndex(pcm, "z");
  if (x_idx < 0 || y_idx < 0 || z_idx < 0) {
    throw std::runtime_error("Point cloud is missing one of the x/y/z fields");
  }
  for (int idx : {x_idx, y_idx, z_idx}) {
    if (pcm.fields[idx].datatype != pcl::PCLPointField::FLOAT32) {
      throw std::runtime_error("FlexCloud only supports float32 x/y/z fields");
    }
  }
  out.x_off = pcm.fields[x_idx].offset;
  out.y_off = pcm.fields[y_idx].offset;
  out.z_off = pcm.fields[z_idx].offset;
  out.point_step = pcm.point_step;
  return out;
}
/**
 * @brief Transform a PCLPointCloud2 in place using Umeyama + Rubber-Sheeting.
 *        Only the x/y/z fields are touched; every other field is preserved.
 *        Points falling outside the rubber-sheet triangulation are removed.
 *        All hardware threads are used.
 */
bool transform::transform_pcd(
  const std::shared_ptr<Umeyama> & umeyama, const std::shared_ptr<Delaunay> & triag,
  pcl::PCLPointCloud2::Ptr & pcm)
{
  if (!pcm || pcm->data.empty()) {
    return false;
  }

  const XYZFieldLayout layout = locate_xyz(*pcm);
  const std::size_t n = static_cast<std::size_t>(pcm->width) * pcm->height;
  const unsigned int num_threads = std::max(1u, std::thread::hardware_concurrency());

  std::vector<std::uint8_t> drop(n, 0);  // 1 = outside triangulation, drop
  std::atomic<std::size_t> processed{0};

  auto worker = [&](std::size_t begin, std::size_t end) {
    for (std::size_t i = begin; i < end; ++i) {
      auto * base = pcm->data.data() + i * layout.point_step;
      float & x = *reinterpret_cast<float *>(base + layout.x_off);
      float & y = *reinterpret_cast<float *>(base + layout.y_off);
      float & z = *reinterpret_cast<float *>(base + layout.z_off);

      Eigen::Vector3d p(x, y, z);
      umeyama->transformPoint(p);
      triag->transformPoint(p);
      if (p.norm() < 1.0e-3) {
        drop[i] = 1;
      } else {
        x = static_cast<float>(p.x());
        y = static_cast<float>(p.y());
        z = static_cast<float>(p.z());
      }
      processed.fetch_add(1, std::memory_order_relaxed);
    }
  };

  std::vector<std::thread> threads;
  threads.reserve(num_threads);
  const std::size_t chunk = (n + num_threads - 1) / num_threads;
  for (unsigned int t = 0; t < num_threads; ++t) {
    const std::size_t begin = t * chunk;
    const std::size_t end = std::min(n, begin + chunk);
    if (begin >= end) break;
    threads.emplace_back(worker, begin, end);
  }
  std::cout << "\033[1;36mStart pcd transformation (" << threads.size() << " threads)\033[0m"
            << std::endl;

  // Progress bar polled from this thread
  const int bar_width = 50;
  int last_decile = -1;
  while (true) {
    const std::size_t done = processed.load(std::memory_order_relaxed);
    const float frac = static_cast<float>(done) / static_cast<float>(n);
    const int decile = static_cast<int>(frac * 10.0f);
    if (decile > last_decile) {
      last_decile = decile;
      const int pos = static_cast<int>(bar_width * frac);
      std::cout << "[";
      for (int i = 0; i < bar_width; ++i) {
        std::cout << (i < pos ? '=' : (i == pos ? '>' : ' '));
      }
      std::cout << "] " << int(frac * 100.0f) << " %" << std::endl;
    }
    if (done >= n) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  for (auto & th : threads) th.join();

  // Drop outliers via PCLPointCloud2-aware ExtractIndices (preserves all fields).
  pcl::PointIndices::Ptr outliers(new pcl::PointIndices);
  outliers->indices.reserve(static_cast<std::size_t>(
    std::count(drop.begin(), drop.end(), std::uint8_t{1})));
  for (std::size_t i = 0; i < n; ++i) {
    if (drop[i]) outliers->indices.push_back(static_cast<int>(i));
  }
  if (!outliers->indices.empty()) {
    pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
    extract.setInputCloud(pcm);
    extract.setIndices(outliers);
    extract.setNegative(true);
    pcl::PCLPointCloud2::Ptr filtered(new pcl::PCLPointCloud2);
    extract.filter(*filtered);
    pcm = filtered;
    std::cout << "\033[1;33mDropped " << outliers->indices.size()
              << " points outside the rubber-sheet triangulation\033[0m" << std::endl;
  }
  std::cout << "\033[1;36mTransformation finished\033[0m" << std::endl;
  return true;
}
}  // namespace flexcloud
