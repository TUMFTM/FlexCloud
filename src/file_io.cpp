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

#include "file_io.hpp"
namespace FlexCloud
{
/**
 * @brief read traj from txt file
 *
 * @param[in] node                - rclcpp::Node:
 *                                  Node reference
 * @param[in] traj_path           - std::string:
 *                                  absolute path to file
 * @param[in] traj_local          - std::vector<ProjPoint>:
 *                                  trajectory as vector of positions with standard dev
 */
bool file_io::read_traj_from_file(
  rclcpp::Node & node, const std::string & traj_path, std::vector<ProjPoint> & traj_local)
{
  const std::string node_name = node.get_parameter("node_name").as_string();
  traj_local.clear();
  if (node.get_parameter("transform_traj").as_bool()) {
    std::vector<GPSPoint> traj_gps;
    Eigen::Vector3d pt_;

    // Read trajectory in GPS format
    double lat, lon, height, lat_stddev, lon_stddev, height_stddev;

    std::ifstream infile(traj_path);
    std::string line;

    while (std::getline(infile, line)) {
      std::istringstream iss(line);

      if (!(iss >> lat >> lon >> height >> lat_stddev >> lon_stddev >> height_stddev)) {
        RCLCPP_ERROR(rclcpp::get_logger(node_name), "Trajectory in wrong format!");
      }
      GPSPoint gps_pt(lat, lon, height, lat_stddev, lon_stddev, height_stddev);
      traj_gps.emplace_back(gps_pt);
    }

    double orig_lat, orig_lon, orig_ele;
    if (node.get_parameter("customZeroPoint").as_bool()) {
      orig_lat = node.get_parameter("zeroLat").as_double();
      orig_lon = node.get_parameter("zeroLon").as_double();
      orig_ele = node.get_parameter("zeroEle").as_double();
    } else {
      orig_lat = traj_gps[0].lat_;
      orig_lon = traj_gps[0].lon_;
      orig_ele = traj_gps[0].ele_;
    }

    std::cout << "\033[33mMap origin: " << orig_lat << " " << orig_lon << " " << orig_ele
              << "\033[0m" << std::endl;

    node.declare_parameter("orig_lat", orig_lat);
    node.declare_parameter("orig_lon", orig_lon);
    node.declare_parameter("orig_ele", orig_ele);

    // initialize GeographicLib origins and ellipsoids
    const GeographicLib::NormalGravity & earth_WGS84 = GeographicLib::NormalGravity::WGS84();

    GeographicLib::Geocentric WGS84ellipsoid =
      GeographicLib::Geocentric(earth_WGS84.EquatorialRadius(), earth_WGS84.Flattening());

    // initialize projection class
    GeographicLib::LocalCartesian proj =
      GeographicLib::LocalCartesian(orig_lat, orig_lon, orig_ele, WGS84ellipsoid);

    for (const auto & gps : traj_gps) {
      ProjPoint pt_proj(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      proj.Forward(gps.lat_, gps.lon_, gps.ele_, pt_proj.pos_[0], pt_proj.pos_[1], pt_proj.pos_[2]);
      pt_proj.stddev_[0] = gps.lat_stddev_;
      pt_proj.stddev_[1] = gps.lon_stddev_;
      pt_proj.stddev_[2] = gps.ele_stddev_;
      traj_local.push_back(pt_proj);
    }
  } else {
    // Read poses
    double x, y, z, x_stddev, y_stddev, z_stddev;

    std::ifstream infile(traj_path);
    std::string line;

    while (std::getline(infile, line)) {
      std::istringstream iss(line);

      if (!(iss >> x >> y >> z >> x_stddev >> y_stddev >> z_stddev)) {
        RCLCPP_ERROR(rclcpp::get_logger(node_name), "Trajectory in wrong format!");
        return false;
      }
      ProjPoint pt(x, y, z, x_stddev, y_stddev, z_stddev);
      traj_local.push_back(pt);
    }
  }
  return true;
}
/**
 * @brief read poses from txt file in KITTI format
 *
 * @param[in] node                - rclcpp::Node:
 *                                  Node reference
 * @param[in] poses_path          - std::string:
 *                                  absolute path to file
 * @param[in] poses               - std::vector<ProjPoint>:
 *                                  trajectory as vector of positions with standard dev
 */
bool file_io::read_poses_SLAM_from_file(
  rclcpp::Node & node, const std::string & poses_path, std::vector<ProjPoint> & poses)
{
  const std::string node_name = node.get_parameter("node_name").as_string();
  poses.clear();
  // Read poses
  double x, y, z;
  float r1, r2, r3, r4, r5, r6, r7, r8, r9;

  std::ifstream infile(poses_path);
  std::string line;

  while (std::getline(infile, line)) {
    std::istringstream iss(line);

    if (!(iss >> r1 >> r2 >> r3 >> x >> r4 >> r5 >> r6 >> y >> r7 >> r8 >> r9 >> z)) {
      RCLCPP_ERROR(rclcpp::get_logger(node_name), "Poses in wrong format!");
      return false;
    }
    ProjPoint pt(x, y, z, 0.0, 0.0, 0.0);
    poses.push_back(pt);
  }
  return true;
}
/**
 * @brief read pcd map from file
 *
 * @param[in] node                - rclcpp::Node:
 *                                  Node reference
 * @param[in] pcd_path            - std::string:
 *                                  absolute path to file
 * @param[in] pcm                 - pcl::PointCloud<pcl::PointXYZ>::Ptr:
 *                                  pointer on pointcloud map
 */
bool file_io::read_pcd_from_file(
  rclcpp::Node & node, const std::string & pcd_path, pcl::PointCloud<pcl::PointXYZI>::Ptr & pcm)
{
  const std::string node_name = node.get_parameter("node_name").as_string();
  // Check for valid filename
  if (pcd_path.substr(pcd_path.length() - 4) != ".pcd") {
    RCLCPP_ERROR(
      rclcpp::get_logger(node_name), "The provided pcd_path does not lead to a .pcd file!");
    return false;
  }

  // Read input cloud based on provided file path
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud) == -1) {
    PCL_ERROR("Couldn't read file Point cloud!\n");
    return -1;
  }
  pcm = cloud;
  return true;
}
/**
 * @brief write pcd map to file
 *
 * @param[in] node                - rclcpp::Node:
 *                                  Node reference
 * @param[in] pcd_out_path        - std::string:
 *                                  absolute path to file
 * @param[in] pcm                 - pcl::PointCloud<pcl::PointXYZ>::Ptr:
 *                                  pointer on pointcloud map
 */
bool file_io::write_pcd_to_path(
  rclcpp::Node & node, const std::string & pcd_out_path,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & pcd_map)
{
  // write to file
  if (node.get_parameter("save_ascii").as_bool()) {
    pcl::io::savePCDFileASCII(pcd_out_path, *pcd_map);
  } else {
    pcl::io::savePCDFileBinary(pcd_out_path, *pcd_map);
  }
  return true;
}
}  // namespace FlexCloud
