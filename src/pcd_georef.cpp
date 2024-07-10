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
namespace FlexCloud
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

  // Initialize publishers
  initialize_publisher();

  // Load trajectory, SLAM poses, ref map and download osm-data
  load_data();

  // Align GPS and SLAM trajectory
  align_traj();

  // Publish trajectory to select control Points for Rubber-Sheeting
  publish_traj();

  // Perform rubber-sheeting to further transform traj and map
  rubber_sheeting();

  // Publish rubber-sheeting data
  publish_rs();

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
 * @brief initialize publishers for visualization in RVIZ
 */
void pcd_georef::initialize_publisher()
{
  rclcpp::QoS durable_qos_pub{1};
  durable_qos_pub.transient_local();

  this->pub_traj_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "tam/traj/traj_markers", durable_qos_pub);
  this->pub_traj_SLAM_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "tam/traj/traj_SLAM_markers", durable_qos_pub);
  this->pub_traj_align_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "tam/traj/traj_align_markers", durable_qos_pub);
  this->pub_traj_rs_markers = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "tam/traj/traj_rs_markers", durable_qos_pub);
  this->pub_rs_geom_markers_tet = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "tam/rs/geom_markers_triag", durable_qos_pub);
  this->pub_rs_geom_markers_cps = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "tam/rs/geom_markers_cps", durable_qos_pub);
  this->pub_pcd_map =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("tam/rs/pcd_map", durable_qos_pub);
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
void pcd_georef::publish_traj()
{
  // Trajectory
  msgs_->linestring2marker_msg(this->traj_proj, this->msg_traj_markers, "WEBGreen", "Trajectory");
  // Poses
  msgs_->linestring2marker_msg(this->traj_SLAM, this->msg_traj_SLAM_markers, "Black", "Poses");
  msgs_->linestring2marker_msg(
    this->traj_align, this->msg_traj_align_markers, "WEBBlueDark", "Poses_align");

  // Publish messages on topics
  this->pub_traj_markers->publish(this->msg_traj_markers);
  this->pub_traj_SLAM_markers->publish(this->msg_traj_SLAM_markers);
  this->pub_traj_align_markers->publish(this->msg_traj_align_markers);
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
void pcd_georef::publish_rs()
{
  // Create messages
  // RS Geometry - Tetrahedra
  msgs_->rs2marker_msg_tet(this->triag_, this->msg_rs_geom_markers_tet);
  this->pub_rs_geom_markers_tet->publish(this->msg_rs_geom_markers_tet);
  // RS Geometry - control point shift
  msgs_->rs2marker_msg_cps(this->control_points, this->msg_rs_geom_markers_cps);
  this->pub_rs_geom_markers_cps->publish(this->msg_rs_geom_markers_cps);

  // Transformed trajectory
  msgs_->linestring2marker_msg(
    this->traj_rs, this->msg_traj_rs_markers, "WEBBlueBright", "traj_rubber_sheeted");
  this->pub_traj_rs_markers->publish(this->msg_traj_rs_markers);

  // PCD map
  if (this->get_parameter("transform_pcd").as_bool()) {
    msgs_->pcd_map2msg(this->pcd_map, this->msg_pcd_map);
    this->pub_pcd_map->publish(this->msg_pcd_map);
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
}  // namespace FlexCloud
/**
 * @brief initialize package
 */
int main(int argc, char ** argv)
{
  // Init
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlexCloud::pcd_georef>("pcd_georef"));
  rclcpp::shutdown();
  return 0;
}
