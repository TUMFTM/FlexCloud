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

namespace flexcloud
{
// Constructor
pcd_georef::pcd_georef(const std::string & name) : Node(name)
{
  this->node_name = name;
  this->declare_parameter("node_name", node_name);

  // Load parameters from command line and config file
  get_params(*this, this->traj_path, this->poses_path, this->pcd_path, this->pcd_out_path);

  // Initialize dimension of transformation
  if (this->get_parameter("dim").as_int() == 2) {
    this->umeyama_ = std::make_shared<Umeyama_2D>();
    this->triag_ = std::make_shared<Delaunay_2D>();
  } else if (this->get_parameter("dim").as_int() == 3) {
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
  std::ifstream infile(this->traj_path);
  if (infile.is_open()) {
    infile.close();
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger(this->node_name), "File to trajectory (traj_path) doesn't exist");
    allValid = false;
  }

  // Check poses_path
  std::ifstream infile2(this->poses_path);
  if (infile2.is_open()) {
    infile2.close();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "File to poses (poses_path) doesn't exist");
    allValid = false;
  }

  // Check pcd path if true
  if (this->get_parameter("transform_pcd").as_bool()) {
    std::ifstream infile2(this->get_parameter("pcd_path").as_string());
    if (infile2.is_open()) {
      infile2.close();
    } else {
      RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "File to pcd (pcd_path) doesn't exist");
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
  if (file_io_->read_traj_from_file(*this, this->traj_path, traj_proj)) {
    std::cout << "\033[1;36m===> Trajectory with " << this->traj_proj.size()
              << " points: Loaded!\033[0m" << std::endl;
    std::cout.precision(17);
    // std::cout << this->get_parameter("orig_lat").as_double() << std::endl <<
    //  this->get_parameter("orig_lon").as_double() << std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during Trajectory loading !!");
  }

  // SLAM poses
  if (file_io_->read_poses_SLAM_from_file(*this, this->poses_path, traj_SLAM)) {
    std::cout << "\033[1;36m===> Poses with " << traj_SLAM.size() << " points: Loaded!\033[0m"
              << std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during Pose loading !!");
  }

  // PCD map
  if (this->get_parameter("transform_pcd").as_bool()) {
    if (file_io_->read_pcd_from_file(*this, this->pcd_path, this->pcd_map)) {
      std::cout << "\033[1;36mPoint Cloud with " << pcd_map->width * pcd_map->height
                << " points: Loaded!\033[0m" << std::endl;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during PCD loading !!");
    }
  }
}
/**
 * @brief align trajectories with Umeyama algorithm
 */
void pcd_georef::align_traj()
{
  // Calculate transformation
  bool bumeyama = transform_.get_umeyama(*this, this->traj_proj, this->traj_SLAM, this->umeyama_);

  // Transform poses and lanelet2 map (3D)
  bool btrans_umeyama =
    transform_.transform_ls_al(this->traj_SLAM, this->traj_align, this->umeyama_);

  if (bumeyama && btrans_umeyama) {
    std::cout << "\033[1;36m===> Trajectory and SLAM poses aligned with Umeyama-algorithm!\033[0m"
              << std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during trajectory alignment !!");
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
  transform_.select_control_points(*this, this->traj_proj, this->traj_align, this->control_points);

  // Calculate triangulation and transformation matrices
  bool btrans_rs =
    transform_.get_rubber_sheeting(*this, this->traj_align, this->control_points, this->triag_);

  // Transform trajectory
  transform_.transform_ls_rs(this->traj_align, this->traj_rs, this->triag_);

  // Transform point cloud map if desired by user
  if (this->get_parameter("transform_pcd").as_bool()) {
    if (this->get_parameter("use_threading").as_bool()) {
      transform_.transform_pcd(
        *this, this->umeyama_, this->triag_, this->pcd_map,
        this->get_parameter("num_cores").as_int());
    } else {
      transform_.transform_pcd(*this, this->umeyama_, this->triag_, this->pcd_map);
    }
  }

  if (btrans_rs) {
    std::cout << "\033[1;36m===> Finished Rubber-Sheeting!\033[0m" << std::endl;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during Rubber-Sheeting !!");
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
  if (this->get_parameter("transform_pcd").as_bool()) {
    this->viz_->pc_map2rerun(this->pcd_map, this->rec_);
  }
}
/**
 * @brief write pcd map to file
 */
void pcd_georef::write_map()
{
  if (this->get_parameter("transform_pcd").as_bool()) {
    if (file_io_->write_pcd_to_path(*this, this->pcd_out_path, this->pcd_map)) {
      std::cout << "\033[1;36mPoint Cloud Map written to " << pcd_out_path << "!\033[0m"
                << std::endl;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger(this->node_name), "!! Error during Map writing !!");
    }
  }
}
/**
 * @brief do evaluation calculations and write to txt-files
 */
void pcd_georef::evaluation()
{
  const std::string path = this->get_parameter("analysis_output_dir").as_string();

  bool btraj_matching = false;
  if (this->get_parameter("analysis_traj_matching").as_bool()) {
    std::vector<double> diff_al;
    std::vector<double> diff_rs;
    btraj_matching = analysis_->traj_matching(
      *this, this->traj_proj, this->traj_SLAM, this->traj_align, this->traj_rs, this->triag_,
      this->control_points, diff_al, diff_rs);
  }

  if (btraj_matching) {
    std::cout << "\033[1;36m===> Analysis calculations saved in " << path << "/ !\033[0m"
              << std::endl;
  }
}
}  // namespace flexcloud
/**
 * @brief initialize package
 */
int main(int argc, char ** argv)
{
  // Init
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<flexcloud::pcd_georef>("pcd_georef"));
  rclcpp::shutdown();
  return 0;
}
