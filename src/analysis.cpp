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

  const std::string traj_matching_dir = "/traj_matching";
  create_output_dir(config, traj_matching_dir);

  write_ls(config, src, traj_matching_dir, "source.txt");
  write_ls(config, target, traj_matching_dir, "target.txt");
  write_ls(config, target_al, traj_matching_dir, "target_al.txt");
  write_ls(config, target_rs, traj_matching_dir, "target_rs.txt");
  write_triag(config, triag, traj_matching_dir, "triag.txt");
  write_cp(config, cps, traj_matching_dir, "controlPoints.txt");
  write_double_vec(config, diff_al, traj_matching_dir, "diff_al.txt");
  write_double_vec(config, diff_rs, traj_matching_dir, "diff_rs.txt");
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
 * @brief create directory from name
 *
 * @param[in] config              - FlexCloudConfig:
 *                                  config struct
 * @param[in] dir_path            - std::string:
 *                                  name of output directory
 */
void analysis::create_output_dir(FlexCloudConfig & config, const std::string & dir_path)
{
  const std::string dir = config.analysis_output_dir + dir_path;
  // Create if not a directory
  if (!std::filesystem::is_directory(dir)) {
    std::filesystem::create_directories(dir);
  }
}
/**
 * @brief write a linestring to .txt file
 *
 * @param[in] config              - FlexCloudConfig:
 *                                  config struct
 * @param[in] ls                  - std::vector<ProjPoint>:
 *                                  linestring
 * @param[in] dir_path            - std::string:
 *                                  name of output directory
 * @param[in] file_name           - std::string:
 *                                  name of output file
 */
void analysis::write_ls(
  FlexCloudConfig & config, const std::vector<ProjPoint> & ls, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path = config.analysis_output_dir + dir_path + "/" + file_name;
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
 * @param[in] config              - FlexCloudConfig:
 *                                  config struct
 * @param[in] lss                 - std::vector<std::vector<ProjPoint>>:
 *                                  vector of linestrings
 * @param[in] dir_path            - std::string:
 *                                  name of output directory
 * @param[in] file_name           - std::string:
 *                                  name of output file
 */
void analysis::write_lss(
  FlexCloudConfig & config, const std::vector<std::vector<ProjPoint>> & lss,
  const std::string & dir_path, const std::string & file_name)
{
  const std::string file_path = config.analysis_output_dir + dir_path + "/" + file_name;
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
 * @param[in] config              - FlexCloudConfig:
 *                                  config struct
 * @param[in] vec                 - std::vector<double>:
 *                                  vector of double values
 * @param[in] dir_path            - std::string:
 *                                  name of output directory
 * @param[in] file_name           - std::string:
 *                                  name of output file
 */
void analysis::write_double_vec(
  FlexCloudConfig & config, const std::vector<double> & vec, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path = config.analysis_output_dir + dir_path + "/" + file_name;
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
 * @param[in] config              - FlexCloudConfig:
 *                                  config struct
 * @param[in] triag               - std::shared_ptr<Delaunay>:
 *                                  pointer to triangulation
 * @param[in] dir_path            - std::string:
 *                                  name of output directory
 * @param[in] file_name           - std::string:
 *                                  name of output file
 */
void analysis::write_triag(
  FlexCloudConfig & config, const std::shared_ptr<Delaunay> & triag, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path = config.analysis_output_dir + dir_path + "/" + file_name;
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
 * @param[in] config              - FlexCloudConfig:
 *                                  config struct
 * @param[in] cps                 - std::vector<ControlPoint>:
 *                                  control points
 * @param[in] dir_path            - std::string:
 *                                  name of output directory
 * @param[in] file_name           - std::string:
 *                                  name of output file
 */
void analysis::write_cp(
  FlexCloudConfig & config, const std::vector<ControlPoint> & cps, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path = config.analysis_output_dir + dir_path + "/" + file_name;
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
