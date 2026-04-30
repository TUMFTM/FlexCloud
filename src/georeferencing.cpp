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
#include <utility>
#include <vector>

#include "CLI/CLI.hpp"
#include "cli/cli_config.hpp"
namespace flexcloud
{
Georeferencing::Georeferencing(config::GeoreferencingConfig cfg) : config_(std::move(cfg))
{
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
  evaluation();

  std::cout << "\033[1;36m===> Done!\033[0m" << std::endl;
}
/**
 * @brief load trajectories and pcd map
 */
void Georeferencing::load_data()
{
  // GPS trajectory
  std::optional<Eigen::Vector3d> origin = std::nullopt;
  if (this->config_.origin.size() == 3) {
    origin =
      Eigen::Vector3d(this->config_.origin[0], this->config_.origin[1], this->config_.origin[2]);
  }
  this->pos_global_ = file_io_->load_positions_file(
    this->config_.positions_path, this->config_.stddev_threshold, origin);
  std::cout << "\033[1;36m===> Trajectory with " << this->pos_global_.size()
            << " points: Loaded!\033[0m" << std::endl;

  // Map poses
  this->poses_ = file_io_->load_poses(this->config_.poses_path);
  std::cout << "\033[1;36m===> Poses with " << this->poses_.size() << " points: Loaded!\033[0m"
            << std::endl;

  // PCD map (preserves all input fields via PCLPointCloud2)
  if (!this->config_.pcd_path.empty()) {
    if (file_io_->load_pcd(this->config_.pcd_path, this->pcd_map_) && this->pcd_map_) {
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

  // Transform poses
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
  // Select control points for rubber-sheeting
  transform_.select_control_points(
    this->config_, this->pos_global_, this->poses_align_, this->control_points_);

  bool btrans_rs = transform_.get_rubber_sheeting(
    this->config_, this->poses_align_, this->control_points_, this->triag_);

  transform_.transform_ls_rs(this->poses_align_, this->poses_rs_, this->triag_);

  // Transform point cloud map if provided by the user
  if (!this->config_.pcd_path.empty()) {
    transform_.transform_pcd(this->umeyama_, this->triag_, this->pcd_map_);

    if (btrans_rs) {
      std::cout << "\033[1;36m===> Finished Rubber-Sheeting!\033[0m" << std::endl;
    } else {
      std::cout << "!! Error during Rubber-Sheeting !!" << std::endl;
    }
  }
}
/**
 * @brief publish results from rubber-sheeting including transformed map
 */
void Georeferencing::visualize_rs()
{
  this->viz_->rs2rerun(this->control_points_, this->triag_, this->rec_, "Blue");
  this->viz_->linestring2rerun(this->poses_rs_, this->rec_, "Orange", "Trajectory_RS");

  if (!this->config_.pcd_path.empty() && this->pcd_map_) {
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

    const bool saved_successfully = file_io_->save_pcd(path, this->pcd_map_);

    if (saved_successfully) {
      std::cout << "\033[1;36mPoint Cloud Map written to " << path << "!\033[0m" << std::endl;
    } else {
      std::cout << "!! Error during Map writing !!" << std::endl;
    }
  }
}
/**
 * @brief do evaluation calculations and write to txt-files
 */
void Georeferencing::evaluation()
{
  // Create output directory
  // Set working directory to current path
  const std::string dir = "./georeferencing_output";
  // Create if not a directory
  if (!std::filesystem::is_directory(dir)) {
    std::filesystem::create_directories(dir);
  }

  // Dump effective config to output directory
  YAML::Node out;
  out["control_points"] = this->config_.control_points;
  out["stddev_threshold"] = this->config_.stddev_threshold;
  out["square_size"] = this->config_.square_size;
  out["origin"] = this->config_.origin;
  out["exclude_ind"] = this->config_.exclude_ind;
  out["shift_ind"] = this->config_.shift_ind;
  out["shift_ind_dist"] = this->config_.shift_ind_dist;
  out["fake_ind"] = this->config_.fake_ind;
  out["fake_ind_dist"] = this->config_.fake_ind_dist;
  out["fake_ind_height"] = this->config_.fake_ind_height;
  std::ofstream fout(dir + "/georeferencing_config.yaml");
  fout << out;
  fout.close();

  // Trajectory matching analysis and export
  this->analysis_->traj_matching(
    dir, this->pos_global_, this->poses_, this->poses_align_, this->poses_rs_, this->triag_,
    this->control_points_);

  std::cout << "\033[1;36m===> Analysis calculations saved in Output directory!\033[0m"
            << std::endl;
}
}  // namespace flexcloud
/**
 * @brief initialize package
 */
int main(int argc, char * argv[])
{
  CLI::App app{
    "Georeference a SLAM trajectory and (optionally) a corresponding point cloud "
    "map by aligning it to a GNSS / reference trajectory using Umeyama and "
    "rubber-sheeting."};
  app.name("georeferencing");

  flexcloud::config::GeoreferencingConfig cfg;
  cfg.add_cli_options(&app);

  app.footer(
    "\nExamples:\n"
    "  # cartesian reference, no point cloud, default parameters\n"
    "  ros2 run flexcloud georeferencing positions_interpolated.txt poses_keyframes.txt\n\n"
    "  # GPS reference, custom origin, transform a point cloud as well\n"
    "  ros2 run flexcloud georeferencing reference.txt poses_keyframes.txt \\\n"
    "      --pcd map.pcd --transform-traj --custom-origin \\\n"
    "      --origin 48.262 11.667 0.0\n\n"
    "  # supply index-based fine-tuning arrays via YAML\n"
    "  ros2 run flexcloud georeferencing reference.txt poses_keyframes.txt \\\n"
    "      --config-file georeferencing.yaml\n");

  CLI11_PARSE(app, argc, argv);

  // Overlay optional YAML for index-based fine-tuning arrays.
  cfg.load_yaml_overlay();

  flexcloud::Georeferencing georef(std::move(cfg));
  return 0;
}
