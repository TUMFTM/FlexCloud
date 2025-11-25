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
 * @param[in] dir                 - std::string:
 *                                  name of output directory
 * @param[in] src                 - std::vector<PointStdDevStamped>:
 *                                  source trajectory
 * @param[in] target              - std::vector<PoseStamped>:
 *                                  target trajectory
 * @param[in] target_al           - std::vector<PoseStamped>:
 *                                  target trajectory after Umeyama trafo
 * @param[in] target_rs           - std::vector<PoseStamped>:
 *                                  target trajectory after rubber-sheeting
 * @param[in] triag               - std::shared_ptr<Delaunay>:
 *                                  pointer to triangulation
 * @param[in] cps                 - std::vector<ControlPoint>:
 *                                  vector of control points
 */
bool analysis::traj_matching(const std::string & dir, const std::vector<PointStdDevStamped> & src,
  const std::vector<PoseStamped> & target, const std::vector<PoseStamped> & target_al,
  const std::vector<PoseStamped> & target_rs, const std::shared_ptr<Delaunay> & triag,
  const std::vector<ControlPoint> & cps)
{
  write_ls(src, dir, "source.txt");
  write_ls(target, dir, "target.txt");
  write_ls(target_al, dir, "target_al.txt");
  write_ls(target_rs, dir, "target_rs.txt");
  write_triag(triag, dir, "triag.txt");
  write_cp(cps, dir, "controlPoints.txt");
  write_double_vec(calc_diff(src, target_al), dir, "diff_al.txt");
  write_double_vec(calc_diff(src, target_rs), dir, "diff_rs.txt");
  return true;
}
/**
 * @brief calculate difference of a target trajectory to a source trajectory
 *
 * @param[in] src                 - std::vector<PointStdDevStamped>:
 *                                  source trajectory
 * @param[in] target              - std::vector<PoseStamped>:
 *                                  target trajectory
 * @return std::vector<double>    - std::vector<double>:
 *                                  difference between trajectories (euclidean distance)
 */
std::vector<double> analysis::calc_diff(
  const std::vector<PointStdDevStamped> & src, const std::vector<PoseStamped> & target)
{
  std::vector<double> diff;
  int i = 0;
  for (const auto & pt : target) {
    double dist = (pt.pose.pose.translation() - src[i].point.pos).norm();
    diff.push_back(dist);
    ++i;
  }
  return diff;
}
/**
 * @brief write a linestring to .txt file
 *
 * @param[in] ls                  - std::vector<PointStdDevStamped>:
 *                                  linestring
 * @param[in] dir_path            - std::string:
 *                                  name of output directory
 * @param[in] file_name           - std::string:
 *                                  name of output file
 */
void analysis::write_ls(
  const std::vector<PointStdDevStamped> & ls, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path = dir_path + "/" + file_name;
  std::ofstream file(file_path);

  if (file.is_open()) {
    for (const auto & pt : ls) {
      file << pt.point.pos.x() << " " << pt.point.pos.y() << " " << pt.point.pos.z() << std::endl;
    }
    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
}
/**
 * @brief write a linestring to .txt file
 *
 * @param[in] ls                  - std::vector<PoseStamped>:
 *                                  linestring
 * @param[in] dir_path            - std::string:
 *                                  name of output directory
 * @param[in] file_name           - std::string:
 *                                  name of output file
 */
void analysis::write_ls(
  const std::vector<PoseStamped> & ls, const std::string & dir_path, const std::string & file_name)
{
  const std::string file_path = dir_path + "/" + file_name;
  std::ofstream file(file_path);

  if (file.is_open()) {
    for (const auto & pose : ls) {
      file << pose.pose.pose.translation().x() << " " << pose.pose.pose.translation().y() << " "
           << pose.pose.pose.translation().z() << std::endl;
    }
    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
}
/**
 * @brief write a linestrings to .txt file
 *
 * @param[in] lss                 - std::vector<std::vector<PointStdDev>>:
 *                                  vector of linestrings
 * @param[in] dir_path            - std::string:
 *                                  name of output directory
 * @param[in] file_name           - std::string:
 *                                  name of output file
 */
void analysis::write_lss(
  const std::vector<std::vector<PointStdDev>> & lss, const std::string & dir_path,
  const std::string & file_name)
{
  const std::string file_path = dir_path + "/" + file_name;
  std::ofstream file(file_path);

  if (file.is_open()) {
    for (const auto & ls : lss) {
      for (const auto & pt : ls) {
        file << pt.pos.x() << " " << pt.pos.y() << " " << pt.pos.z() << " ";
      }
      file << std::endl;
    }
    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
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

  std::vector<std::vector<Eigen::Vector3d>> vertices = triag->getVertices();

  if (file.is_open()) {
    for (const auto & tet : vertices) {
      for (const Eigen::Vector3d & p : tet) {
        file << p.x() << " " << p.y() << " " << p.z() << " ";
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
      Eigen::Vector3d src = cpt.source;
      Eigen::Vector3d target = cpt.target;
      file << src(0) << " " << src(1) << " " << src(2) << " " << target(0) << " " << target(1)
           << " " << target(2) << std::endl;
    }
    file.close();
  } else {
    std::cout << "\033[1;31m!! Unable to open " << file_path << " !!\033[0m" << std::endl;
  }
}
}  // namespace flexcloud
