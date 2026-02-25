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

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
namespace flexcloud
{
/**
 * @brief Load position frames from a directory
 *
 * @param[in] directory           - std::string:
 *                                  absolute path to directory
 * @param[in] stddev_threshold    - float:
 *                                  threshold for standard deviation
 * @return std::vector<PointStdDevStamped>:
 *                                  vector of position frames
 */
std::vector<PointStdDevStamped> file_io::load_positions_dir(
  const std::string & directory, const float stddev_threshold)
{
  std::vector<PointStdDevStamped> pos_frames{};
  std::cout << "Loading position frames from " << directory << std::endl;

  // Count number of files
  int count = 0;
  boost::filesystem::path dirPath(directory);
  try {
    for (const auto & entry : boost::filesystem::directory_iterator(dirPath)) {
      if (boost::filesystem::is_regular_file(entry) && entry.path().extension() == ".txt") {
        ++count;
      }
    }
  } catch (const boost::filesystem::filesystem_error & ex) {
    std::cerr << "Error accessing directory: " << ex.what() << std::endl;
  }

  for (boost::filesystem::directory_iterator it(dirPath), it_end; it != it_end; ++it) {
    if (it->path().extension() != ".txt") {
      continue;
    }

    std::string filePath = directory + "/" + it->path().stem().string() + ".txt";
    std::ifstream inputFile(filePath);
    if (!inputFile.is_open()) {
      std::cerr << "Unable to open file" << std::endl;
      return pos_frames;
    }

    std::string line;
    double x_pos, y_pos, z_pos, x_stddev, y_stddev, z_stddev;
    if (std::getline(inputFile, line)) {
      std::istringstream iss(line);

      if (!(iss >> x_pos >> y_pos >> z_pos >> x_stddev >> y_stddev >> z_stddev)) {
        std::cerr << "Error during extraction of x, y, z, x_stdded, y_stddev and z_stddev values"
                  << std::endl;
      }
    } else {
      std::cerr << "File is empty" << std::endl;
    }
    inputFile.close();

    std::int64_t stamp_sec = 0;
    std::int64_t stamp_usec = 0;
    char underscore;
    std::stringstream sst(boost::filesystem::path(filePath).filename().string());
    sst >> stamp_sec >> underscore >> stamp_usec;

    // Check of stddev of lat and lon too high. If yes don't add frame to vector
    if (std::sqrt(std::pow(x_stddev, 2) + std::pow(y_stddev, 2)) > stddev_threshold) {
      std::cout << "\033[31m!! Skipped frame due to standard deviation exceeding limits !!\033[0m"
                << std::endl;
      continue;
    }
    PointStdDev point(x_pos, y_pos, z_pos, x_stddev, y_stddev, z_stddev);
    pos_frames.push_back(PointStdDevStamped(point, stamp_sec, stamp_usec));
  }

  // Sort frames based on timestamps (smallest TS first)
  std::sort(
    pos_frames.begin(), pos_frames.end(),
    [](const PointStdDevStamped & a, const PointStdDevStamped & b) { return a.stamp < b.stamp; });

  std::cout << "Loaded " << pos_frames.size() << " global position frames" << std::endl;
  return pos_frames;
}
/**
 * @brief read traj from txt file
 *
 * @param[in] config              - GeoreferencingConfig:
 *                                  config struct
 * @param[in] path                 - std::string:
 *                                  absolute path to file
 */
std::vector<PointStdDevStamped> file_io::load_positions(
  const std::string & path, GeoreferencingConfig & config)
{
  std::vector<PointStdDevStamped> points_local{};
  if (config.transform_traj) {
    std::vector<PointStdDev> points_gps{};

    // Read trajectory in GPS format
    double stamp, lat, lon, height, lat_stddev, lon_stddev, height_stddev;

    std::ifstream infile(path);
    std::string line;

    while (std::getline(infile, line)) {
      std::istringstream iss(line);

      if (!(iss >> stamp >> lat >> lon >> height >> lat_stddev >> lon_stddev >> height_stddev)) {
        std::cout << "Trajectory in wrong format!" << std::endl;
      }
      PointStdDev gps_pt(lat, lon, height, lat_stddev, lon_stddev, height_stddev);
      points_gps.emplace_back(gps_pt);
    }

    Eigen::Vector3d orig{0.0, 0.0, 0.0};
    if (config.custom_origin) {
      orig.x() = config.origin[0];
      orig.y() = config.origin[1];
      orig.z() = config.origin[2];
    } else {
      orig = points_gps[0].pos;
    }

    std::cout << "\033[33mMap origin: " << orig.x() << " " << orig.y() << " " << orig.z()
              << "\033[0m" << std::endl;

    config.origin = {orig.x(), orig.y(), orig.z()};

    // initialize GeographicLib origins and ellipsoids
    const GeographicLib::NormalGravity & earth_WGS84 = GeographicLib::NormalGravity::WGS84();

    GeographicLib::Geocentric WGS84ellipsoid =
      GeographicLib::Geocentric(earth_WGS84.EquatorialRadius(), earth_WGS84.Flattening());

    // initialize projection class
    GeographicLib::LocalCartesian proj =
      GeographicLib::LocalCartesian(orig.x(), orig.y(), orig.z(), WGS84ellipsoid);

    for (const auto & gps : points_gps) {
      PointStdDev pt_proj(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      proj.Forward(
        gps.pos.y(), gps.pos.x(), gps.pos.z(), pt_proj.pos.x(), pt_proj.pos.y(), pt_proj.pos.z());
      pt_proj.stddev.x() = gps.stddev.x();
      pt_proj.stddev.y() = gps.stddev.y();
      pt_proj.stddev.z() = gps.stddev.z();
      points_local.push_back(PointStdDevStamped(pt_proj, static_cast<int64_t>(stamp * 1e9)));
    }
  } else {
    // Read poses
    double stamp, x, y, z, x_stddev, y_stddev, z_stddev;

    std::ifstream infile(path);
    std::string line;

    while (std::getline(infile, line)) {
      std::istringstream iss(line);

      if (!(iss >> stamp >> x >> y >> z >> x_stddev >> y_stddev >> z_stddev)) {
        std::cout << "Trajectory in wrong format!" << std::endl;
        throw std::invalid_argument("Trajectory in wrong format!");
      }
      points_local.push_back(PointStdDevStamped(
        PointStdDev(x, y, z, x_stddev, y_stddev, z_stddev), static_cast<int64_t>(stamp * 1e9)));
    }
  }
  return points_local;
}
/**
 * @brief Load glim odometry from a file
 */
std::vector<PoseStamped> file_io::load_poses(const std::string & file_path)
{
  std::vector<PoseStamped> poses{};
  std::ifstream input_file(file_path);
  if (!input_file.is_open()) {
    std::cerr << "Unable to open file" << std::endl;
    return poses;
  }

  std::string line;
  // Read poses
  double x, y, z, stamp;
  float qx, qy, qz, qw;

  while (std::getline(input_file, line)) {
    std::istringstream iss(line);

    if (!(iss >> stamp >> x >> y >> z >> qx >> qy >> qz >> qw)) {
      std::cerr << "Error during extraction of odometry pose" << std::endl;
    }
    Pose pose(x, y, z, qx, qy, qz, qw);
    poses.push_back(PoseStamped(pose, stamp));
  }
  input_file.close();
  return poses;
}
/**
 * @brief read pcd map from file
 *
 * @param[in] pcd_path            - std::string:
 *                                  absolute path to file
 * @param[in] pcm                 - pcl::PointCloud<PointT>::Ptr:
 *                                  pointer on pointcloud map
 */
template <typename PointT>
bool file_io::load_pcd(const std::string & pcd_path, typename pcl::PointCloud<PointT>::Ptr & pcm)
{
  // Check for valid filename
  if (pcd_path.substr(pcd_path.length() - 4) != ".pcd") {
    std::cout << "The provided pcd_path does not lead to a .pcd file!" << std::endl;
    return false;
  }

  // Read input cloud based on provided file path
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(pcd_path, *cloud) == -1) {
    PCL_ERROR("Couldn't read file Point cloud!\n");
    return -1;
  }
  pcm = cloud;
  return true;
}
/**
 * @brief save position frames to file
 *
 * @param[in] filename            - std::string:
 *                                  absolute path to file
 * @param[in] keyframes           - std::vector<std::shared_ptr<PoseStamped>>:
 *                                  vector of keyframes
 */
bool file_io::save_positions(
  const std::string & filename, const std::vector<PointStdDevStamped> & positions)
{
  std::ofstream ofs(filename);
  if (!ofs) {
    return false;
  }
  // clang-format off
  for (const auto & pos : positions) {
    ofs << std::fixed << std::setprecision(14) << pos.stamp * 1.0e-9 << " "
        << std::fixed << std::setprecision(14) << pos.point.pos.x() << " "
        << std::fixed << std::setprecision(14) << pos.point.pos.y() << " "
        << std::fixed << std::setprecision(14) << pos.point.pos.z() << " "
        << std::fixed << std::setprecision(14) << pos.point.stddev.x() << " "
        << std::fixed << std::setprecision(14) << pos.point.stddev.y() << " "
        << std::fixed << std::setprecision(14) << pos.point.stddev.z() << " " << std::endl;
  }
  // clang-format on
  ofs.close();

  return true;
}
/**
 * @brief save kitti odometry to file
 *
 * @param[in] filename           - std::string:
 *                                  absolute path to file
 * @param[in] keyframes          - std::vector<std::shared_ptr<OdometryFrame>>:
 *                                  vector of keyframes
 */
bool file_io::save_poses(const std::string & filename, const std::vector<PoseStamped> & poses)
{
  std::ofstream ofs(filename);
  if (!ofs) {
    return false;
  }
  for (size_t i = 0; i < poses.size(); i++) {
    const Eigen::Quaterniond rotation(poses[i].pose.pose.rotation());
    const Eigen::Vector3d translation = poses[i].pose.pose.translation();
    ofs << poses[i].stamp << " " << translation.x() << " " << translation.y() << " "
        << translation.z() << " " << rotation.x() << " " << rotation.y() << " " << rotation.z()
        << " " << rotation.w() << std::endl;
  }
  ofs.close();

  return true;
}
/**
 * @brief write pcd map to file
 *
 * @param[in] config              - GeoreferencingConfig:
 *                                  config struct
 * @param[in] pcd_out_path        - std::string:
 *                                  absolute path to file
 * @param[in] pcm                 - pcl::PointCloud<PointT>::Ptr:
 *                                  pointer on pointcloud map
 */
template <typename PointT>
bool file_io::save_pcd(
  const std::string & pcd_out_path, const typename pcl::PointCloud<PointT>::Ptr & pcd_map)
{
  // write to file
  pcl::io::savePCDFileBinary(pcd_out_path, *pcd_map);
  return true;
}
// Explicit instantiations for point types used
template bool file_io::load_pcd<pcl::PointXYZI>(
  const std::string & pcd_path, typename pcl::PointCloud<pcl::PointXYZI>::Ptr & pcm);
template bool file_io::save_pcd<pcl::PointXYZI>(
  const std::string & pcd_out_path, const typename pcl::PointCloud<pcl::PointXYZI>::Ptr & pcd_map);
template bool file_io::load_pcd<PointXYZIL>(
  const std::string & pcd_path, typename pcl::PointCloud<PointXYZIL>::Ptr & pcm);
template bool file_io::save_pcd<PointXYZIL>(
  const std::string & pcd_out_path, const typename pcl::PointCloud<PointXYZIL>::Ptr & pcd_map);
}  // namespace flexcloud
