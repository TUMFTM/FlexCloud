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
 * @param [in] origin             - std::optional<Eigen::Vector3d>:
 *                                  optional origin for ENU projection
 * @return std::vector<PointStdDevStamped>:
 *                                  vector of position frames
 */
std::vector<PointStdDevStamped> file_io::load_positions_dir(
  const std::string & directory, const float stddev_threshold,
  const std::optional<Eigen::Vector3d> & origin)
{
  // Setup projection if origin provided
  ENUProjection proj;
  if (origin.has_value()) {
    proj.set_origin(origin.value());
    std::cout << "Using ENU projection with origin at (" << origin.value().transpose() << ")" << std::endl;
  } else {
    std::cout << "No origin provided, interpreting positions as already in ENU" << std::endl;
  }
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
    Eigen::Vector3d xyz(x_pos, y_pos, z_pos);
    if (origin.has_value()) {
      // Interpret as (lat, lon, alt) and project to local ENU.
      xyz = proj.forward(x_pos, y_pos, z_pos);
    }
    PointStdDev point(xyz.x(), xyz.y(), xyz.z(), x_stddev, y_stddev, z_stddev);
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
 * @brief Load positions from a single txt file with one position per line.
 *
 * @param[in] file_path           - std::string:
 *                                  absolute path to file
 * @param[in] stddev_threshold    - float:
 *                                  threshold for standard deviation
 * @param [in] origin             - std::optional<Eigen::Vector3d>:
 *                                  optional origin for ENU projection
 * @return std::vector<PointStdDevStamped>:
 *                                  vector of position frames
 */
std::vector<PointStdDevStamped> file_io::load_positions_file(
  const std::string & file_path, const float stddev_threshold,
  const std::optional<Eigen::Vector3d> & origin)
{
  // Setup projection if origin provided
  ENUProjection proj;
  if (origin.has_value()) {
    proj.set_origin(origin.value());
    std::cout << "Using ENU projection with origin at (" << origin.value().transpose() << ")" << std::endl;
  } else {
    std::cout << "No origin provided, interpreting positions as already in ENU" << std::endl;
  }
  std::vector<PointStdDevStamped> pos_frames{};
  std::cout << "Loading position frames from " << file_path << std::endl;

  std::ifstream infile(file_path);
  if (!infile.is_open()) {
    std::cerr << "Unable to open file: " << file_path << std::endl;
    return pos_frames;
  }

  std::string line;
  std::size_t skipped = 0;
  while (std::getline(infile, line)) {
    if (line.empty()) continue;
    std::istringstream iss(line);
    double stamp, x, y, z, x_stddev, y_stddev, z_stddev;
    if (!(iss >> stamp >> x >> y >> z >> x_stddev >> y_stddev >> z_stddev)) {
      std::cerr << "Skipping malformed line: " << line << std::endl;
      continue;
    }
    if (std::sqrt(std::pow(x_stddev, 2) + std::pow(y_stddev, 2)) > stddev_threshold) {
      ++skipped;
      continue;
    }
    Eigen::Vector3d xyz(x, y, z);
    if (origin.has_value()) {
      // Interpret as (lat, lon, alt) and project to local ENU.
      xyz = proj.forward(x, y, z);
    }
    pos_frames.push_back(PointStdDevStamped(
      PointStdDev(xyz.x(), xyz.y(), xyz.z(), x_stddev, y_stddev, z_stddev),
      static_cast<int64_t>(stamp * 1e9)));
  }

  std::sort(
    pos_frames.begin(), pos_frames.end(),
    [](const PointStdDevStamped & a, const PointStdDevStamped & b) { return a.stamp < b.stamp; });

  if (skipped > 0) {
    std::cout << "\033[31m!! Skipped " << skipped
              << " frames due to standard deviation exceeding limits !!\033[0m" << std::endl;
  }
  std::cout << "Loaded " << pos_frames.size() << " global position frames" << std::endl;
  return pos_frames;
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
 * @brief Read a PCD map into a pcl::PCLPointCloud2 (preserves all fields).
 *
 * @param[in] pcd_path            - std::string:
 *                                  absolute path to PCD file
 * @param[out] pcm                 - pcl::PCLPointCloud2::Ptr:
 *                                  point cloud map; allocated and filled by this function
 * @return true if executed
 */
bool file_io::load_pcd(const std::string & pcd_path, pcl::PCLPointCloud2::Ptr & pcm)
{
  if (pcd_path.size() < 4 || pcd_path.substr(pcd_path.length() - 4) != ".pcd") {
    std::cout << "The provided pcd_path does not lead to a .pcd file!" << std::endl;
    return false;
  }
  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
  if (pcl::io::loadPCDFile(pcd_path, *cloud) == -1) {
    PCL_ERROR("Couldn't read file Point cloud!\n");
    return false;
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
 * @brief Write a pcl::PCLPointCloud2 map to file (preserves all fields).
 * @param[in] pcd_out_path        - std::string:
 *                                  absolute path to output PCD file
 * @param[in] pcd_map             - pcl::PCLPointCloud2::Ptr:
 *                                  point cloud map to save
 * @return true if executed
 */
bool file_io::save_pcd(const std::string & pcd_out_path, const pcl::PCLPointCloud2::Ptr & pcd_map)
{
  pcl::PCDWriter writer;
  // Compressed binary preserves arbitrary fields without loss.
  return writer.writeBinaryCompressed(pcd_out_path, *pcd_map) == 0;
}
}  // namespace flexcloud
