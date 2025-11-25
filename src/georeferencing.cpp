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

#include "georeferencing.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace flexcloud
{
// Constructor
// pcd_georef package constructor
Georeferencing::Georeferencing(
  const std::string & config_path, const std::string & pos_global_path,
  const std::string & poses_path, const std::string & pcd_path = "")
{
  // Load config from file
  YAML::Node config = YAML::LoadFile(config_path);

  // Set parameters
  this->config_.pos_global_path = pos_global_path;
  this->config_.poses_path = poses_path;
  this->config_.pcd_path = pcd_path;
  this->config_.transform_traj = config["transform_traj"].as<bool>();
  this->config_.rs_num_controlPoints = config["rs_num_controlPoints"].as<int>();
  this->config_.stddev_threshold = config["stddev_threshold"].as<double>();
  this->config_.square_size = config["square_size"].as<std::vector<double>>();
  this->config_.exclude_ind = config["exclude_ind"].as<std::vector<int64_t>>();
  this->config_.shift_ind = config["shift_ind"].as<std::vector<int64_t>>();
  this->config_.shift_ind_dist = config["shift_ind_dist"].as<std::vector<double>>();
  this->config_.fake_ind = config["fake_ind"].as<std::vector<int64_t>>();
  this->config_.fake_ind_dist = config["fake_ind_dist"].as<std::vector<double>>();
  this->config_.fake_ind_height = config["fake_ind_height"].as<std::vector<double>>();
  this->config_.num_cores = config["num_cores"].as<int>();
  this->config_.custom_origin = config["custom_origin"].as<bool>();
  this->config_.origin = config["origin"].as<std::vector<double>>();

  // Check if all paths contain data
  if (!paths_valid()) return;

  // Load data and point cloud map
  load_data();

  // Align global positions and poses with Umeyama algorithm
  align_traj();

  // Visualize aligned trajectories
  visualize_traj();

  // Perform rubber-sheeting to further align poses to global positions
  rubber_sheeting();

  // Publish rubber-sheeting data
  visualize_rs();

  // Write pcd to file
  save_map();

  // Analysis and save
  evaluation(config);

  std::cout << "\033[1;36m===> Done!\033[0m" << std::endl;
}
/**
 * @brief check if all necessary paths exist
 */
bool Georeferencing::paths_valid()
{
  bool valid = true;

  auto check_file = [&valid](const std::string & path, const std::string & name) {
    std::ifstream file(path);
    if (!file.is_open()) {
      std::cout << "File to " << name << " (" << name << "_path) doesn't exist" << std::endl;
      valid = false;
    }
  };

  check_file(this->config_.pos_global_path, "trajectory");
  check_file(this->config_.poses_path, "poses");

  if (this->config_.pcd_path != "") {
    check_file(this->config_.pcd_path, "pcd");
  }

  return valid;
}
/**
 * @brief load trajectories and pcd map
 */
void Georeferencing::load_data()
{
  // GPS trajectory
  this->pos_global_ = file_io_->load_positions(this->config_.pos_global_path, this->config_);
  std::cout << "\033[1;36m===> Trajectory with " << this->pos_global_.size()
            << " points: Loaded!\033[0m" << std::endl;

  // Map poses
  this->poses_ = file_io_->load_poses(this->config_.poses_path);
  std::cout << "\033[1;36m===> Poses with " << this->poses_.size() << " points: Loaded!\033[0m"
            << std::endl;

  // PCD map
  if (this->config_.pcd_path != "") {
    if (file_io_->load_pcd(this->config_.pcd_path, this->pcd_map_)) {
      std::cout << "\033[1;36mPoint Cloud with " << pcd_map_->width * pcd_map_->height
                << " points: Loaded!\033[0m" << std::endl;
    } else {
      std::cout << "!! Error during PCD loading !!" << std::endl;
    }
  }
}
/**
 * @brief align trajectories with Umeyama algorithm
 */
void Georeferencing::align_traj()
{
  // Calculate transformation
  bool bumeyama = transform_.get_umeyama(this->pos_global_, this->poses_, this->umeyama_);

  // Transform poses and lanelet2 map (3D)
  bool btrans_umeyama =
    transform_.transform_ls_al(this->poses_, this->poses_align_, this->umeyama_);

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
void Georeferencing::visualize_traj()
{
  // Spawn a rerun stream
  this->rec_.spawn().exit_on_failure();

  // // Trajectory
  viz_->linestring2rerun(this->pos_global_, this->rec_, "WEBGreen", "Trajectory");
  viz_->linestring2rerun(this->poses_, this->rec_, "Black", "Trajectory_SLAM");
  viz_->linestring2rerun(this->poses_align_, this->rec_, "WEBBlueDark", "Trajectory_align");
}
/**
 * @brief apply automatic or manual rubber-sheet trafo and transform map
 *        and trajectories
 */
void Georeferencing::rubber_sheeting()
{
  // Get controlpoints from RVIZ
  transform_.select_control_points(
    this->config_, this->pos_global_, this->poses_align_, this->control_points_);

  // Calculate triangulation and transformation matrices
  bool btrans_rs = transform_.get_rubber_sheeting(
    this->config_, this->poses_align_, this->control_points_, this->triag_);

  // Transform trajectory
  transform_.transform_ls_rs(this->poses_align_, this->poses_rs_, this->triag_);

  // Transform point cloud map if desired by user
  if (this->config_.pcd_path != "") {
    transform_.transform_pcd(this->umeyama_, this->triag_, this->pcd_map_, this->config_.num_cores);
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
void Georeferencing::visualize_rs()
{
  // Visualize in rerun
  this->viz_->rs2rerun(this->control_points_, this->triag_, this->rec_, "Blue");
  this->viz_->linestring2rerun(this->poses_rs_, this->rec_, "Orange", "Trajectory_RS");

  // PCD map
  if (this->config_.pcd_path != "") {
    this->viz_->pc_map2rerun(this->pcd_map_, this->rec_);
  }
}
/**
 * @brief write pcd map to file
 */
void Georeferencing::save_map()
{
  if (this->config_.pcd_path != "") {
    std::string path =
      this->config_.pcd_path.substr(0, this->config_.pcd_path.find_last_of("\\/")) + "/georef_" +
      this->config_.pcd_path.substr(this->config_.pcd_path.find_last_of("/\\") + 1);

    if (file_io_->save_pcd(path, this->pcd_map_)) {
      std::cout << "\033[1;36mPoint Cloud Map written to " << path << "!\033[0m" << std::endl;
    } else {
      std::cout << "!! Error during Map writing !!" << std::endl;
    }
  }
}
/**
 * @brief do evaluation calculations and write to txt-files
 * 
 * @param[in] config               - YAML::Node:
 *                                  configuration node
 */
void Georeferencing::evaluation(const YAML::Node & config)
{
  // Create output directory
  // Set working directory to current path
  const std::string dir = "./georeferencing_output";
  // Create if not a directory
  if (!std::filesystem::is_directory(dir)) {
    std::filesystem::create_directories(dir);
  }

  // Dump config file to output directory
  std::ofstream fout(dir + "/georeferencing_config.yaml");
  fout << config;
  fout.close();

  // Trajectory matching analysis and export
  this->analysis_->traj_matching(dir, this->pos_global_, this->poses_, this->poses_align_, this->poses_rs_,
    this->triag_, this->control_points_);

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
    flexcloud::Georeferencing georef(argv[1], argv[2], argv[3], "");
  } else if (argc == 5) {
    flexcloud::Georeferencing georef(argv[1], argv[2], argv[3], argv[4]);
  } else {
    std::cerr << "Too many arguments" << std::endl;
    return 1;
  }
  return 0;
}
