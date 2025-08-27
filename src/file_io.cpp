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
 * @return std::vector<PosFrame>:
 *                                  vector of position frames
 */
std::vector<PosFrame> file_io::load_pos_frames(
  const std::string & directory, const float stddev_threshold)
{
  std::vector<PosFrame> pos_frames{};
  std::cout << "Loading position frames from " << directory << std::endl;
  boost::filesystem::directory_iterator dir_itr(directory);
  boost::filesystem::directory_iterator end;

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

  for (dir_itr; dir_itr != end; dir_itr++) {
    if (dir_itr->path().extension() != ".txt") {
      continue;
    }

    std::string filePath = directory + "/" + dir_itr->path().stem().string() + ".txt";
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

    PosFrame frame(stamp_sec, stamp_usec, x_pos, y_pos, z_pos, x_stddev, y_stddev, z_stddev);
    pos_frames.push_back(frame);
  }

  // Sort frames based on timestamps (smallest TS first)
  std::sort(pos_frames.begin(), pos_frames.end(), [](const PosFrame & a, const PosFrame & b) {
    return a.get_timestamp() < b.get_timestamp();
  });

  std::cout << "Loaded " << pos_frames.size() << " global position frames" << std::endl;
  return pos_frames;
}
/**
 * @brief Load kitti odometry from a file
 */
std::vector<Eigen::Isometry3d> file_io::load_kitti_odom(const std::string & file_path)
{
  std::vector<Eigen::Isometry3d> poses;
  std::ifstream input_file(file_path);
  if (!input_file.is_open()) {
    std::cerr << "Unable to open file" << std::endl;
    return poses;
  }

  std::string line;
  // Read poses
  double x, y, z;
  float r1, r2, r3, r4, r5, r6, r7, r8, r9;

  while (std::getline(input_file, line)) {
    std::istringstream iss(line);

    if (!(iss >> r1 >> r2 >> r3 >> x >> r4 >> r5 >> r6 >> y >> r7 >> r8 >> r9 >> z)) {
      std::cerr << "Error during extraction of odometry pose" << std::endl;
    }
    Eigen::Matrix3d rotation;
    rotation << r1, r2, r3, r4, r5, r6, r7, r8, r9;

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = rotation;
    pose.translation() << x, y, z;

    poses.push_back(pose);
  }
  input_file.close();
  return poses;
}
/**
 * @brief Load glim odometry from a file
 */
std::vector<Eigen::Isometry3d> file_io::load_glim_odom(
  const std::string & file_path, std::vector<double> & timestamps)
{
  std::vector<Eigen::Isometry3d> poses;
  timestamps.clear();
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
    Eigen::Matrix3d rotation;
    rotation = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = rotation;
    pose.translation() << x, y, z;

    poses.push_back(pose);
    timestamps.push_back(stamp);
  }
  input_file.close();
  return poses;
}
/**
 * @brief read traj from txt file
 *
 * @param[in] config              - FlexCloudConfig:
 *                                  config struct
 * @param[in] traj_path           - std::string:
 *                                  absolute path to file
 * @param[in] traj_local          - std::vector<ProjPoint>:
 *                                  trajectory as vector of positions with standard dev
 */
bool file_io::read_traj_from_file(
  FlexCloudConfig & config, const std::string & traj_path, std::vector<ProjPoint> & traj_local)
{
  traj_local.clear();
  if (config.transform_traj) {
    std::vector<GPSPoint> traj_gps;
    Eigen::Vector3d pt_;

    // Read trajectory in GPS format
    double lat, lon, height, lat_stddev, lon_stddev, height_stddev;

    std::ifstream infile(traj_path);
    std::string line;

    while (std::getline(infile, line)) {
      std::istringstream iss(line);

      if (!(iss >> lat >> lon >> height >> lat_stddev >> lon_stddev >> height_stddev)) {
        std::cout << "Trajectory in wrong format!" << std::endl;
      }
      GPSPoint gps_pt(lat, lon, height, lat_stddev, lon_stddev, height_stddev);
      traj_gps.emplace_back(gps_pt);
    }

    double orig_lat, orig_lon, orig_ele;
    if (config.customZeroPoint) {
      orig_lat = config.zeroPoint[0];
      orig_lon = config.zeroPoint[1];
      orig_ele = config.zeroPoint[2];
    } else {
      orig_lat = traj_gps[0].lat_;
      orig_lon = traj_gps[0].lon_;
      orig_ele = traj_gps[0].ele_;
    }

    std::cout << "\033[33mMap origin: " << orig_lat << " " << orig_lon << " " << orig_ele
              << "\033[0m" << std::endl;

    config.zeroPoint = {orig_lat, orig_lon, orig_ele};

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
        std::cout << "Trajectory in wrong format!" << std::endl;
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
 * @param[in] config              - FlexCloudConfig:
 *                                  config struct
 * @param[in] poses_path          - std::string:
 *                                  absolute path to file
 * @param[in] poses               - std::vector<ProjPoint>:
 *                                  trajectory as vector of positions with standard dev
 */
bool file_io::read_poses_SLAM_from_file(
  FlexCloudConfig & config, const std::string & poses_path, std::vector<ProjPoint> & poses)
{
  poses.clear();
  // Read poses
  double x, y, z;
  float r1, r2, r3, r4, r5, r6, r7, r8, r9;

  std::ifstream infile(poses_path);
  std::string line;

  while (std::getline(infile, line)) {
    std::istringstream iss(line);

    if (!(iss >> r1 >> r2 >> r3 >> x >> r4 >> r5 >> r6 >> y >> r7 >> r8 >> r9 >> z)) {
      std::cout << "Poses in wrong format!" << std::endl;
      return false;
    }
    ProjPoint pt(x, y, z, 0.0, 0.0, 0.0);
    poses.push_back(pt);
  }
  return true;
}
/**
 * @brief Load pcd point clouds from a directory
 *
 * @param[in] directory           - std::string:
 *                                  absolute path to directory
 * @return std::vector<std::string>:
 *                                  vector of pcd filenames
 */
std::vector<std::string> file_io::load_clouds(const std::string & directory)
{
  std::vector<std::string> pcd_filenames{};
  // Load filenames from directory
  boost::filesystem::directory_iterator dir_itr(directory);
  boost::filesystem::directory_iterator end;
  // Count number of files
  int count = 0;
  boost::filesystem::path dirPath(directory);
  try {
    for (const auto & entry : boost::filesystem::directory_iterator(dirPath)) {
      if (boost::filesystem::is_regular_file(entry) && entry.path().extension() == ".pcd") {
        ++count;
      }
    }
  } catch (const boost::filesystem::filesystem_error & ex) {
    std::cerr << "Error accessing directory: " << ex.what() << std::endl;
  }

  for (dir_itr; dir_itr != end; dir_itr++) {
    if (dir_itr->path().extension() != ".pcd") {
      continue;
    }

    std::string file_path = directory + "/" + dir_itr->path().stem().string() + ".pcd";
    pcd_filenames.push_back(file_path);
  }
  std::sort(pcd_filenames.begin(), pcd_filenames.end());
  return pcd_filenames;
}
/**
 * @brief read pcd map from file
 *
 * @param[in] config              - FlexCloudConfig:
 *                                  config struct
 * @param[in] pcd_path            - std::string:
 *                                  absolute path to file
 * @param[in] pcm                 - pcl::PointCloud<pcl::PointXYZ>::Ptr:
 *                                  pointer on pointcloud map
 */
bool file_io::read_pcd_from_file(
  FlexCloudConfig & config, const std::string & pcd_path,
  pcl::PointCloud<pcl::PointXYZI>::Ptr & pcm)
{
  // Check for valid filename
  if (pcd_path.substr(pcd_path.length() - 4) != ".pcd") {
    std::cout << "The provided pcd_path does not lead to a .pcd file!" << std::endl;
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
 * @brief save kitti odometry to file
 *
 * @param[in] filename            - std::string:
 *                                  absolute path to file
 * @param[in] keyframes           - std::vector<std::shared_ptr<OdometryFrame>>:
 *                                  vector of keyframes
 */
bool file_io::save_graph(
  const std::string & filename, const std::vector<std::shared_ptr<OdometryFrame>> & keyframes)
{
  std::ofstream ofs(filename);
  if (!ofs) {
    return false;
  }

  // Write vertices
  for (size_t i = 0; i < keyframes.size(); i++) {
    ofs << "VERTEX_SE3:QUAT " << i << " ";
    const Eigen::Vector3d translation = keyframes[i]->pose.translation();
    const Eigen::Quaterniond quaternion(keyframes[i]->pose.rotation());

    ofs << translation.x() << " " << translation.y() << " " << translation.z() << " "
        << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " "
        << quaternion.w();
    ofs << std::endl;
  }
  // Fix first vertex
  ofs << "FIX 0" << std::endl;

  // Write edges
  for (size_t i = 0; i < keyframes.size() - 1; i++) {
    const auto & delta_pose = keyframes[i]->pose.inverse() * keyframes[i + 1]->pose;
    const Eigen::Vector3d translation = delta_pose.translation();
    const Eigen::Quaterniond quaternion(delta_pose.rotation());

    Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
    inf.block<3, 3>(0, 0) *= 10.0;
    inf.block<3, 3>(3, 3) *= 20.0;

    ofs << "EDGE_SE3:QUAT " << i << " " << i + 1 << " ";
    // Write delta pose
    ofs << translation.x() << " " << translation.y() << " " << translation.z() << " "
        << quaternion.x() << " " << quaternion.y() << " " << quaternion.z() << " " << quaternion.w()
        << " ";
    // Write information matrix
    for (int row = 0; row < inf.rows(); ++row) {
      for (int col = row; col < inf.cols(); ++col) {
        ofs << inf(row, col) << " ";
      }
    }
    ofs << std::endl;
  }

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
bool file_io::save_kitti(
  const std::string & filename, const std::vector<std::shared_ptr<OdometryFrame>> & keyframes)
{
  std::ofstream ofs(filename);
  if (!ofs) {
    return false;
  }
  for (size_t i = 0; i < keyframes.size(); i++) {
    const Eigen::Matrix3d rotation = keyframes[i]->pose.rotation();
    const Eigen::Vector3d translation = keyframes[i]->pose.translation();
    ofs << rotation(0, 0) << " " << rotation(0, 1) << " " << rotation(0, 2) << " "
        << translation.x() << " " << rotation(1, 0) << " " << rotation(1, 1) << " "
        << rotation(1, 2) << " " << translation.y() << " " << rotation(2, 0) << " "
        << rotation(2, 1) << " " << rotation(2, 2) << " " << translation.z() << std::endl;
  }
  ofs.close();

  return true;
}
/**
 * @brief save keyframes to directory
 *
 * @param[in] directory           - std::string:
 *                                  absolute path to directory
 * @param[in] keyframes           - std::vector<std::shared_ptr<OdometryFrame>>:
 *                                  vector of keyframes
 * @param[in] downsample          - float:
 *                                  downsample factor
 */
bool file_io::save_keyframes(
  const std::string & directory, const std::vector<std::shared_ptr<OdometryFrame>> & keyframes,
  const float downsample)
{
  for (size_t i = 0; i < keyframes.size(); i++) {
    std::string keyframe_directory = (boost::format("%s/%06d") % directory % i).str();
    boost::filesystem::create_directories(keyframe_directory);

    boost::filesystem::copy_file(keyframes[i]->raw_cloud_path, keyframe_directory + "/raw.pcd");
    pcl::io::savePCDFileBinary(keyframe_directory + "/cloud.pcd", *keyframes[i]->cloud(downsample));

    std::ofstream ofs(keyframe_directory + "/data");
    if (!ofs) {
      return false;
    }

    ofs << "stamp " << keyframes[i]->stamp_sec << " " << keyframes[i]->stamp_nsec << std::endl;
    ofs << "estimate" << std::endl << keyframes[i]->pose.matrix() << std::endl;
    ofs << "odom " << std::endl << keyframes[i]->pose.matrix() << std::endl;
    ofs << "id " << i << std::endl;
  }

  return true;
}
bool file_io::save_pos_frames(
  const std::string & filename, const std::vector<PosFrame> & pos_keyframes)
{
  std::ofstream ofs(filename);
  if (!ofs) {
    return false;
  }
  for (size_t i = 0; i < pos_keyframes.size(); i++) {
    ofs << std::fixed << std::setprecision(14) << pos_keyframes.at(i).x_pos << " " << std::fixed
        << std::setprecision(14) << pos_keyframes.at(i).y_pos << " " << std::fixed
        << std::setprecision(14) << pos_keyframes.at(i).z_pos << " " << std::fixed
        << std::setprecision(14) << pos_keyframes.at(i).x_stddev << " " << std::fixed
        << std::setprecision(14) << pos_keyframes.at(i).y_stddev << " " << std::fixed
        << std::setprecision(14) << pos_keyframes.at(i).z_stddev << " " << std::endl;
  }
  ofs.close();

  return true;
}
/**
 * @brief Accumulate all keyframes and save to single pcd file
 *
 * @param[in] path                - std::string:
 *                                  absolute path to file
 * @param[in] keyframes           - std::vector<std::shared_ptr<OdometryFrame>>:
 *                                  vector of keyframes
 * @param[in] downsample          - float:
 *                                  downsample factor
 */
bool file_io::save_accumulated_cloud(
  const std::string & path, const std::vector<std::shared_ptr<OdometryFrame>> & keyframes,
  const float downsample)
{
  // Create output cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  // Loop through all frames
  for (size_t i = 0; i < keyframes.size(); i++) {
    // Create temporary cloud for transformed points
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // Transform cloud using frame's pose
    pcl::transformPointCloud(
      *keyframes[i]->cloud(downsample), *transformed_cloud, keyframes[i]->pose.matrix());

    // Accumulate points
    *accumulated_cloud += *transformed_cloud;
    std::cout << "Accumulated " << accumulated_cloud->size() << " points" << std::endl;
  }
  std::cout << "Total accumulated points: " << accumulated_cloud->size() << std::endl;
  pcl::io::savePCDFileBinary(path, *accumulated_cloud);
  return true;
}
/**
 * @brief write pcd map to file
 *
 * @param[in] config              - FlexCloudConfig:
 *                                  config struct
 * @param[in] pcd_out_path        - std::string:
 *                                  absolute path to file
 * @param[in] pcm                 - pcl::PointCloud<pcl::PointXYZ>::Ptr:
 *                                  pointer on pointcloud map
 */
bool file_io::write_pcd_to_path(
  const std::string & pcd_out_path, const pcl::PointCloud<pcl::PointXYZI>::Ptr & pcd_map)
{
  // write to file
  pcl::io::savePCDFileBinary(pcd_out_path, *pcd_map);
  return true;
}
}  // namespace flexcloud
