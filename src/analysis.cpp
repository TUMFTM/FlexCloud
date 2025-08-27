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

#include "analysis.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>
namespace flexcloud
{
/**
 * @brief write all data relevant for evaluation of trajectory matching
 *
 * @param[in] config              - FlexCloudConfig:
 *                                  config struct
 * @param[in] src                 - std::vector<ProjPoint>:
 *                                  source trajectory
 * @param[in] target              - std::vector<ProjPoint>:
 *                                  target trajectory
 * @param[in] target_al           - std::vector<ProjPoint>:
 *                                  target trajectory after Umeyama trafo
 * @param[in] target_rs           - std::vector<ProjPoint>:
 *                                  target trajectory after rubber-sheeting
 * @param[in] triag               - std::shared_ptr<Delaunay>:
 *                                  pointer to triangulation
 * @param[in] cps                 - std::vector<ControlPoint>:
 *                                  vector of control points
 * @param[in] diff_al             - std::vector<double>:
 *                                  difference of aligned trajectory to source trajectory
 * @param[in] diff_rs             - std::vector<double>:
 *                                  difference of rubber-sheeted trajectory to source trajectory
 */
bool analysis::traj_matching(
  FlexCloudConfig & config, const std::vector<ProjPoint> & src,
  const std::vector<ProjPoint> & target, const std::vector<ProjPoint> & target_al,
  const std::vector<ProjPoint> & target_rs, const std::shared_ptr<Delaunay> & triag,
  const std::vector<ControlPoint> & cps, std::vector<double> & diff_al,
  std::vector<double> & diff_rs)
{
  diff_al.clear();
  diff_rs.clear();

  calc_diff(config, src, target_al, diff_al);
  calc_diff(config, src, target_rs, diff_rs);

  // Extract path from output path
  std::string path = config.pcd_out_path;
  std::string::size_type pos = path.find_last_of("/");
  if (pos != std::string::npos) {
    path = path.substr(0, pos);
  }
  const std::string dir = path + "/traj_matching";
  // Create if not a directory
  if (!std::filesystem::is_directory(dir)) {
    std::filesystem::create_directories(dir);
  }
  save_config(config, dir, "config.txt");
  write_ls(src, dir, "source.txt");
  write_ls(target, dir, "target.txt");
  write_ls(target_al, dir, "target_al.txt");
  write_ls(target_rs, dir, "target_rs.txt");
  write_triag(triag, dir, "triag.txt");
  write_cp(cps, dir, "controlPoints.txt");
  write_double_vec(diff_al, dir, "diff_al.txt");
  write_double_vec(diff_rs, dir, "diff_rs.txt");
  return true;
}
/**
 * @brief calculate difference of a target trajectory to a source trajectory
 *
 * @param[in] config              - FlexCloudConfig:
 *                                  config struct
 * @param[in] src                 - std::vector<ProjPoint>:
 *                                  source trajectory
 * @param[in] target              - std::vector<ProjPoint>:
 *                                  target trajectory
 * @param[in] diff                - std::vector<double>:
 *                                  difference between trajectories (euclidean distance)
 */
void analysis::calc_diff(
  FlexCloudConfig & config, const std::vector<ProjPoint> & src,
  const std::vector<ProjPoint> & target, std::vector<double> & diff)
{
  diff.clear();
  int i = 0;
  for (const auto & pt : target) {
    if (config.dim == 2) {
      // Only 2D distance
      double dist =
        (Eigen::Vector2d(pt.pos_(0), pt.pos_(1)) - Eigen::Vector2d(src[i].pos_(0), src[i].pos_(1)))
          .norm();
      diff.push_back(dist);
      ++i;
    } else if (config.dim == 3) {
      double dist = (pt.pos_ - src[i].pos_).norm();
      diff.push_back(dist);
      ++i;
    } else {
      throw std::invalid_argument("Invalid Dimension!");
    }
  }
}
/**
 * @brief Save FlexCloudConfig to a text file
 * @param config The configuration to save
 * @param filepath Path to save the configuration file
 * @return true if successful, false otherwise
 */
void analysis::save_config(
  const FlexCloudConfig & config, const std::string & dir_path, const std::string & file_name)
{
  const std::string file_path = dir_path + "/" + file_name;
  std::ofstream file(file_path);

  if (file.is_open()) {
    // Set precision for floating-point values
    file << std::fixed << std::setprecision(6);

    // Write basic string paths
    file << "traj_path=" << config.traj_path << std::endl;
    file << "poses_path=" << config.poses_path << std::endl;
    file << "pcd_path=" << config.pcd_path << std::endl;
    file << "pcd_out_path=" << config.pcd_out_path << std::endl;

    // Write dimension
    file << "dim=" << config.dim << std::endl;

    // Write trajectory alignment parameters
    file << "transform_traj=" << (config.transform_traj ? "true" : "false") << std::endl;
    file << "rs_num_controlPoints=" << config.rs_num_controlPoints << std::endl;
    file << "stddev_threshold=" << config.stddev_threshold << std::endl;

    // Write vector fields with comma separation
    file << "square_size=";
    for (size_t i = 0; i < config.square_size.size(); ++i) {
      file << config.square_size[i];
      if (i < config.square_size.size() - 1) file << ",";
    }
    file << std::endl;

    // Write PCD georeferencing parameters
    file << "transform_pcd=" << (config.transform_pcd ? "true" : "false") << std::endl;

    file << "exclude_ind=";
    for (size_t i = 0; i < config.exclude_ind.size(); ++i) {
      file << config.exclude_ind[i];
      if (i < config.exclude_ind.size() - 1) file << ",";
    }
    file << std::endl;

    file << "shift_ind=";
    for (size_t i = 0; i < config.shift_ind.size(); ++i) {
      file << config.shift_ind[i];
      if (i < config.shift_ind.size() - 1) file << ",";
    }
    file << std::endl;

    file << "shift_ind_dist=";
    for (size_t i = 0; i < config.shift_ind_dist.size(); ++i) {
      file << config.shift_ind_dist[i];
      if (i < config.shift_ind_dist.size() - 1) file << ",";
    }
    file << std::endl;

    file << "fake_ind=";
    for (size_t i = 0; i < config.fake_ind.size(); ++i) {
      file << config.fake_ind[i];
      if (i < config.fake_ind.size() - 1) file << ",";
    }
    file << std::endl;

    file << "fake_ind_dist=";
    for (size_t i = 0; i < config.fake_ind_dist.size(); ++i) {
      file << config.fake_ind_dist[i];
      if (i < config.fake_ind_dist.size() - 1) file << ",";
    }
    file << std::endl;

    // Write threading parameters
    file << "use_threading=" << (config.use_threading ? "true" : "false") << std::endl;
    file << "num_cores=" << config.num_cores << std::endl;

    // Write zero point parameters
    file << "customZeroPoint=" << (config.customZeroPoint ? "true" : "false") << std::endl;

    file << "zeroPoint=";
    for (size_t i = 0; i < config.zeroPoint.size(); ++i) {
      file << config.zeroPoint[i];
      if (i < config.zeroPoint.size() - 1) file << ",";
    }
    file << std::endl;
    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
  return;
}
/**
 * @brief write a linestring to .txt file
 *
 * @param[in] ls                  - std::vector<ProjPoint>:
 *                                  linestring
 * @param[in] dir_path            - std::string:
 *                                  name of output directory
 * @param[in] file_name           - std::string:
 *                                  name of output file
 */
void analysis::write_ls(
  const std::vector<ProjPoint> & ls, const std::string & dir_path, const std::string & file_name)
{
  const std::string file_path = dir_path + "/" + file_name;
  std::ofstream file(file_path);

  if (file.is_open()) {
    for (const auto & pt : ls) {
      file << pt.pos_(0) << " " << pt.pos_(1) << " " << pt.pos_(2) << std::endl;
    }
    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
}
/**
 * @brief write a linestrings to .txt file
 *
 * @param[in] lss                 - std::vector<std::vector<ProjPoint>>:
 *                                  vector of linestrings
 * @param[in] dir_path            - std::string:
 *                                  name of output directory
 * @param[in] file_name           - std::string:
 *                                  name of output file
 */
void analysis::write_lss(
  const std::vector<std::vector<ProjPoint>> & lss, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path = dir_path + "/" + file_name;
  std::ofstream file(file_path);

  if (file.is_open()) {
    for (const auto & ls : lss) {
      for (const auto & pt : ls) {
        file << pt.pos_(0) << " " << pt.pos_(1) << " " << pt.pos_(2) << " ";
      }
      file << std::endl;
    }
    file.close();
  } else {
    std::cout << "\033[1;31m!! UnabDelaunay triagle to open " << file_path << " !!\033[0m"
              << std::endl;
  }
}
/**
 * @brief write a double vector to .txt file
 *
 * @param[in] vec                 - std::vector<double>:
 *                                  vector of double values
 * @param[in] dir_path            - std::string:
 *                                  name of output directory
 * @param[in] file_name           - std::string:
 *                                  name of output file
 */
void analysis::write_double_vec(
  const std::vector<double> & vec, const std::string & dir_path, const std::string & file_name)
{
  const std::string file_path = dir_path + "/" + file_name;
  std::ofstream file(file_path);

  if (file.is_open()) {
    for (const auto & diff : vec) file << diff << std::endl;

    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
}
/**
 * @brief write triangulation vertices to file
 *
 * @param[in] triag               - std::shared_ptr<Delaunay>:
 *                                  pointer to triangulation
 * @param[in] dir_path            - std::string:
 *                                  name of output directory
 * @param[in] file_name           - std::string:
 *                                  name of output file
 */
void analysis::write_triag(
  const std::shared_ptr<Delaunay> & triag, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path = dir_path + "/" + file_name;
  std::ofstream file(file_path);

  std::vector<std::vector<ProjPoint>> vertices = triag->getVertices();

  if (file.is_open()) {
    for (const auto & tet : vertices) {
      for (const ProjPoint & p : tet) {
        file << p.pos_(0) << " " << p.pos_(1) << " " << p.pos_(2) << " ";
      }
      file << std::endl;
    }
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
}
/**
 * @brief write controlpoints to file
 *
 * @param[in] cps                 - std::vector<ControlPoint>:
 *                                  control points
 * @param[in] dir_path            - std::string:
 *                                  name of output directory
 * @param[in] file_name           - std::string:
 *                                  name of output file
 */
void analysis::write_cp(
  const std::vector<ControlPoint> & cps, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path = dir_path + "/" + file_name;
  std::ofstream file(file_path);

  if (file.is_open()) {
    for (const auto & cpt : cps) {
      Eigen::Vector3d src = cpt.get_source_point();
      Eigen::Vector3d target = cpt.get_target_point();
      file << src(0) << " " << src(1) << " " << src(2) << " " << target(0) << " " << target(1)
           << " " << target(2) << std::endl;
    }
    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
}
}  // namespace flexcloud
