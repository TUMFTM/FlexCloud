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

#include "pcd_georef.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
namespace flexcloud
{
// Constructor
// pcd_georef package constructor
pcd_georef::pcd_georef(
  const std::string & config_path, const std::string & ref_path, const std::string & slam_path,
  const std::string & pcd_path, const std::string & pcd_out_path = "pcd_georef.pcd")
{
  // Load config from file
  YAML::Node config = YAML::LoadFile(config_path);

  // Set parameters
  this->config_.traj_path = ref_path;
  this->config_.poses_path = slam_path;
  this->config_.pcd_path = pcd_path;
  this->config_.pcd_out_path = pcd_out_path;
  this->config_.dim = config["dim"].as<int>();
  this->config_.transform_traj = config["transform_traj"].as<bool>();
  this->config_.rs_num_controlPoints = config["rs_num_controlPoints"].as<int>();
  this->config_.stddev_threshold = config["stddev_threshold"].as<double>();
  this->config_.square_size = config["square_size"].as<std::vector<double>>();
  this->config_.transform_pcd = config["transform_pcd"].as<bool>();
  this->config_.exclude_ind = config["exclude_ind"].as<std::vector<int64_t>>();
  this->config_.shift_ind = config["shift_ind"].as<std::vector<int64_t>>();
  this->config_.shift_ind_dist = config["shift_ind_dist"].as<std::vector<double>>();
  this->config_.fake_ind = config["fake_ind"].as<std::vector<int64_t>>();
  this->config_.fake_ind_dist = config["fake_ind_dist"].as<std::vector<double>>();
  this->config_.fake_ind_height = config["fake_ind_height"].as<std::vector<double>>();
  this->config_.use_threading = config["use_threading"].as<bool>();
  this->config_.num_cores = config["num_cores"].as<int>();
  this->config_.customZeroPoint = config["customZeroPoint"].as<bool>();
  this->config_.zeroPoint = config["zeroPoint"].as<std::vector<double>>();

  // Initialize dimension of transformation
  if (this->config_.dim == 2) {
    this->umeyama_ = std::make_shared<Umeyama_2D>();
    this->triag_ = std::make_shared<Delaunay_2D>();
  } else if (this->config_.dim == 3) {
    this->umeyama_ = std::make_shared<Umeyama_3D>();
    this->triag_ = std::make_shared<Delaunay_3D>();
  } else {
    throw std::invalid_argument("Invalid Dimension!");
  }

  // Check if all paths contain data
  if (!paths_valid()) return;

  // Load trajectory, SLAM poses, ref map and download osm-data
  load_data();

  // Align GPS and SLAM trajectory
  align_traj();

  // Publish trajectory to select control Points for Rubber-Sheeting
  visualize_traj();

  // Perform rubber-sheeting to further transform traj and map
  rubber_sheeting();

  // Publish rubber-sheeting data
  visualize_rs();

  // Write pcd to file
  write_map();

  // Analysis and save
  evaluation();

  std::cout << "\033[1;36m===> Done!\033[0m" << std::endl;
}
/**
 * @brief check if all necessary paths exist
 */
bool pcd_georef::paths_valid()
{
  bool allValid = true;

  // Check traj_path
  std::ifstream infile(this->config_.traj_path);
  if (infile.is_open()) {
    infile.close();
  } else {
    std::cout << "File to trajectory (traj_path) doesn't exist" << std::endl;
    allValid = false;
  }

  // Check poses_path
  std::ifstream infile2(this->config_.poses_path);
  if (infile2.is_open()) {
    infile2.close();
  } else {
    std::cout << "File to poses (poses_path) doesn't exist" << std::endl;
    allValid = false;
  }

  // Check pcd path if true
  if (this->config_.transform_pcd) {
    std::ifstream infile2(this->config_.pcd_path);
    if (infile2.is_open()) {
      infile2.close();
    } else {
      std::cout << "File to pcd (pcd_path) doesn't exist" << std::endl;
      allValid = false;
    }
  }

  return allValid;
}
/**
 * @brief load trajectories and pcd map
 */
void pcd_georef::load_data()
{
  // GPS trajectory
  if (file_io_->read_traj_from_file(this->config_, this->config_.traj_path, traj_proj)) {
    std::cout << "\033[1;36m===> Trajectory with " << this->traj_proj.size()
              << " points: Loaded!\033[0m" << std::endl;
    std::cout.precision(17);
    // std::cout << this->get_parameter("orig_lat").as_double() << std::endl <<
    //  this->get_parameter("orig_lon").as_double() << std::endl;
  } else {
    std::cout << "!! Error during Trajectory loading !!" << std::endl;
  }

  // SLAM poses
  if (file_io_->read_poses_SLAM_from_file(this->config_, this->config_.poses_path, traj_SLAM)) {
    std::cout << "\033[1;36m===> Poses with " << traj_SLAM.size() << " points: Loaded!\033[0m"
              << std::endl;
  } else {
    std::cout << "!! Error during Pose loading !!" << std::endl;
  }

  // PCD map
  if (this->config_.transform_pcd) {
    if (file_io_->read_pcd_from_file(this->config_, this->config_.pcd_path, this->pcd_map)) {
      std::cout << "\033[1;36mPoint Cloud with " << pcd_map->width * pcd_map->height
                << " points: Loaded!\033[0m" << std::endl;
    } else {
      std::cout << "!! Error during PCD loading !!" << std::endl;
    }
  }
}
/**
 * @brief align trajectories with Umeyama algorithm
 */
void pcd_georef::align_traj()
{
  // Calculate transformation
  bool bumeyama =
    transform_.get_umeyama(this->config_, this->traj_proj, this->traj_SLAM, this->umeyama_);

  // Transform poses and lanelet2 map (3D)
  bool btrans_umeyama =
    transform_.transform_ls_al(this->traj_SLAM, this->traj_align, this->umeyama_);

  if (bumeyama && btrans_umeyama) {
    std::cout << "\033[1;36m===> Trajectory and SLAM poses aligned with Umeyama-algorithm!\033[0m"
              << std::endl;
  } else {
    std::cout << "!! Error during trajectory alignment !!" << std::endl;
  }
}
/**
 * @brief publish resulting trajectories from alignment
 */
void pcd_georef::visualize_traj()
{
  // Spawn a rerun stream
  this->rec_.spawn().exit_on_failure();

  // // Trajectory
  viz_->linestring2rerun(this->traj_proj, this->rec_, "WEBGreen", "Trajectory");
  viz_->linestring2rerun(this->traj_SLAM, this->rec_, "Black", "Trajectory_SLAM");
  viz_->linestring2rerun(this->traj_align, this->rec_, "WEBBlueDark", "Trajectory_align");
}
/**
 * @brief apply automatic or manual rubber-sheet trafo and transform map
 *        and trajectories
 */
void pcd_georef::rubber_sheeting()
{
  // Get controlpoints from RVIZ
  transform_.select_control_points(
    this->config_, this->traj_proj, this->traj_align, this->control_points);

  // Calculate triangulation and transformation matrices
  bool btrans_rs = transform_.get_rubber_sheeting(
    this->config_, this->traj_align, this->control_points, this->triag_);

  // Transform trajectory
  transform_.transform_ls_rs(this->traj_align, this->traj_rs, this->triag_);

  // Transform point cloud map if desired by user
  if (this->config_.transform_pcd) {
    if (this->config_.use_threading) {
      transform_.transform_pcd(
        this->umeyama_, this->triag_, this->pcd_map, this->config_.num_cores);
    } else {
      transform_.transform_pcd(this->umeyama_, this->triag_, this->pcd_map);
    }
  }

  if (btrans_rs) {
    std::cout << "\033[1;36m===> Finished Rubber-Sheeting!\033[0m" << std::endl;
  } else {
    std::cout << "!! Error during Rubber-Sheeting !!" << std::endl;
  }
}
/**
 * @brief publish results from rubber-sheeting including transformed map
 */
void pcd_georef::visualize_rs()
{
  // Visualize in rerun
  this->viz_->rs2rerun(this->control_points, this->triag_, this->rec_, "Blue");
  this->viz_->linestring2rerun(this->traj_rs, this->rec_, "Orange", "Trajectory_RS");

  // PCD map
  if (this->config_.transform_pcd) {
    this->viz_->pc_map2rerun(this->pcd_map, this->rec_);
  }
}
/**
 * @brief write pcd map to file
 */
void pcd_georef::write_map()
{
  if (this->config_.transform_pcd) {
    if (file_io_->write_pcd_to_path(this->config_.pcd_out_path, this->pcd_map)) {
      std::cout << "\033[1;36mPoint Cloud Map written to " << config_.pcd_out_path << "!\033[0m"
                << std::endl;
    } else {
      std::cout << "!! Error during Map writing !!" << std::endl;
    }
  }
}
/**
 * @brief do evaluation calculations and write to txt-files
 */
void pcd_georef::evaluation()
{
  std::vector<double> diff_al;
  std::vector<double> diff_rs;
  bool saved = this->analysis_->traj_matching(
    this->config_, this->traj_proj, this->traj_SLAM, this->traj_align, this->traj_rs, this->triag_,
    this->control_points, diff_al, diff_rs);

  std::cout << "\033[1;36m===> Analysis calculations saved in Output directory!\033[0m"
            << std::endl;
}
}  // namespace flexcloud
/**
 * @brief initialize package
 */
int main(int argc, char * argv[])
{
  // Check the number of arguments
  if (argc < 4) {
    // Tell the user how to run the program
    std::cerr << "Usage: " << argv[0]
              << " <config_path> <reference_path> <slam_path> <(optional) pcd_path> <(optional) "
                 "pcd_out_path>"
              << std::endl;
    return 1;
  }
  if (argc == 4) {
    flexcloud::pcd_georef pcd_georef(argv[1], argv[2], argv[3], "", "");
  } else if (argc == 5) {
    flexcloud::pcd_georef pcd_georef(argv[1], argv[2], argv[3], argv[4], "");
  } else if (argc == 6) {
    flexcloud::pcd_georef pcd_georef(argv[1], argv[2], argv[3], argv[4], argv[5]);
  } else {
    std::cerr << "Too many arguments" << std::endl;
    return 1;
  }
  return 0;
}
