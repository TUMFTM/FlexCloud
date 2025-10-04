/*
 * TUM Autonomous Motorsport Georeferencing Tool
 * Copyright (C) 2024 Maximilian Leitenstern, Marko Alten
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
// based on: https://github.com/koide3/interactive_slam/blob/master/src/odometry2graph.cpp

#include "keyframe_interpolation.hpp"

#include <algorithm>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"
namespace flexcloud
{
/**
 * @brief Constructor for KeyframeInterpolation class without accumulation of clouds
 */
KeyframeInterpolation::KeyframeInterpolation(
  const std::string & config_path, const std::string & pos_dir, const std::string & odom_path,
  const std::string & dst_directory)
{
  this->globalMaxTimeDiff_ = 0.0f;
  this->set_params(config_path);

  // Load position frames
  load(pos_dir, odom_path);

  // Interpolate keyframes
  select_keyframes();

  // Save keyframes
  save(dst_directory);
}
/**
 * @brief Constructor for KeyframeInterpolation class with accumulation of clouds
 */
KeyframeInterpolation::KeyframeInterpolation(
  const std::string & config_path, const std::string & pos_dir, const std::string & odom_path,
  const std::string & pcd_dir, const std::string & dst_directory)
{
  this->globalMaxTimeDiff_ = 0.0f;
  this->set_params(config_path);

  // Load position frames
  load(pos_dir, odom_path, pcd_dir);

  // Interpolate keyframes
  select_keyframes();

  // Save keyframes
  save(dst_directory);
}
/**
 * @brief Load frames from a directory
 *
 * @param[in] pos_dir             - std::string:
 *                                 absolute path to directory
 * @param[in] kitti_path          - std::string:
 *                                path to kitti odometry
 */
void KeyframeInterpolation::load(
  const std::string & pos_dir, const std::string & odom_path)
{
  // Load position frames
  this->pos_frames_.clear();
  this->pos_frames_ = file_io_->load_pos_frames(pos_dir, this->stddev_threshold_);

  // Load odometry frames
  this->frames_.clear();
  // Load kitti odometry file
  std::cout << "Loading odometry poses from " << odom_path << std::endl;
  std::vector<Eigen::Isometry3d> poses{};
  std::vector<double> timestamps{};
  if (this->odom_format_ == "kitti") {
    poses = file_io_->load_kitti_odom(odom_path);
  } else if (this->odom_format_ == "glim") {
    poses = file_io_->load_glim_odom(odom_path, timestamps);
  } else {
    throw std::runtime_error(
      "Unknown odometry format: " + this->odom_format_ + ". Supported formats are: kitti, glim");
    return;
  }

  // Load pcd files and creates frames
  for (size_t i = 0; i < poses.size(); ++i) {
    this->frames_.push_back(OdometryFrame::from_pose_and_timestamp(poses[i], timestamps[i]));
  }
  std::cout << "Loaded " << this->frames_.size() << " odometry frames" << std::endl;
}
/**
 * @brief Load frames from a directory
 *
 * @param[in] pos_dir             - std::string:
 *                                 absolute path to directory
 * @param[in] kitti_path          - std::string:
 *                                path to kitti odometry
 * * @param[in] pcd_dir             - std::string:
 * *                                 absolute path to directory
 */
void KeyframeInterpolation::load(
  const std::string & pos_dir, const std::string & odom_path,
  const std::string & pcd_dir)
{
  // Load position frames
  this->pos_frames_.clear();
  this->pos_frames_ = file_io_->load_pos_frames(pos_dir, this->stddev_threshold_);

  // Load odometry frames
  this->frames_.clear();
  // Load kitti odometry file
  std::cout << "Loading odometry poses from " << odom_path << std::endl;
  std::vector<Eigen::Isometry3d> poses{};
  std::vector<double> timestamps{};
  if (this->odom_format_ == "kitti") {
    poses = file_io_->load_kitti_odom(odom_path);
  } else if (this->odom_format_ == "glim") {
    poses = file_io_->load_glim_odom(odom_path, timestamps);
  } else {
    throw std::runtime_error(
      "Unknown odometry format: " + this->odom_format_ + ". Supported formats are: kitti, glim");
    return;
  }
  // // Load pcd cloud filenames
  // std::cout << "Loading pcd-clouds from " << pcd_dir << std::endl;
  // std::vector<std::string> pcd_filenames = file_io_->load_clouds(pcd_dir);
  // // Check if sizes match
  // if (pcd_filenames.size() != poses.size()) {
  //   std::cerr << "Number of pcd files and poses do not match: " << pcd_filenames.size() << " vs "
  //             << poses.size() << std::endl;
  //   return;
  // }

  // Load pcd files and creates frames
  for (size_t i = 0; i < poses.size(); ++i) {
    // auto frame = OdometryFrame::load(pcd_filenames[i], poses[i], this->downsample_resolution_);
    // if (frame == nullptr) {
    //   continue;
    // }
    // this->frames_.push_back(frame);
    this->frames_.push_back(OdometryFrame::from_pose_and_timestamp(poses[i], timestamps[i]));
  }
  std::cout << "Loaded " << this->frames_.size() << " odometry frames" << std::endl;
}
/**
 * @brief Save everything to directory
 *
 * @param[in] dst_directory       - std::string:
 *                                 absolute path to directory
 */
bool KeyframeInterpolation::save(
  const std::string & dst_directory) const
{
  if (this->keyframes_.empty()) {
    return false;
  }

  // Create directory
  if (!boost::filesystem::is_directory(dst_directory)) {
    boost::filesystem::create_directories(dst_directory);
  }

  if (!file_io_->save_kitti(dst_directory + "/kitti_poses.txt", this->keyframes_)) {
    return false;
  }

  if (!file_io_->save_pos_frames(dst_directory + "/poseData.txt", this->pos_keyframes_)) {
    return false;
  }

  // Save graph for kitti format and accumulated cloud for glim format
  if (this->odom_format_ == "kitti") {
    if (!file_io_->save_graph(dst_directory + "/graph.g2o", this->keyframes_)) {
      return false;
    }
    if (!file_io_->save_keyframes(dst_directory, this->keyframes_, this->downsample_resolution_)) {
      return false;
    }
  } else if (this->odom_format_ == "glim") {
    // if (!file_io_->save_accumulated_cloud(
    //       dst_directory + "/map.pcd", this->keyframes_, this->downsample_resolution_)) {
    //   return false;
    // }
  }
  std::cout << "Everything saved successfully" << std::endl;
  return true;
}
/**
 * @brief Select keyframes
 */
void KeyframeInterpolation::select_keyframes()
{
  if (this->frames_.empty() || this->pos_frames_.empty()) {
    return;
  }

  this->keyframes_.clear();
  this->pos_keyframes_.clear();

  // Handle first frame
  this->keyframes_.push_back(this->frames_.front());
  if (this->interpolate_) {
    /* Interpolate position of frame */
    this->pos_keyframes_.push_back(interpolate_pos(this->frames_.front()));
  } else {
    /* Search GPS frame for first frame */
    this->pos_keyframes_.push_back(search_closest(this->frames_.front()));
  }

  // Set keyframes
  for (const auto & frame : this->frames_) {
    const auto & last_keyframe_pose = this->keyframes_.back()->pose;
    const auto & current_frame_pose = frame->pose;

    Eigen::Isometry3d delta = last_keyframe_pose.inverse() * current_frame_pose;
    double delta_x = delta.translation().norm();
    double delta_angle = Eigen::AngleAxisd(delta.linear()).angle();

    if (delta_x > this->keyframe_delta_x_ || delta_angle > this->keyframe_delta_angle_) {
      this->keyframes_.push_back(frame);

      if (this->interpolate_) {
        // Interpolate position of frame
        this->pos_keyframes_.push_back(interpolate_pos(frame));
      } else {
        // search for corresponding pos file that is closest (time) to frame
        this->pos_keyframes_.push_back(search_closest(frame));
      }
    }
  }

  if (!this->interpolate_) {
    // Output max time difference that occured during keyframe selection
    std::cout << "\033[1;32m"
              << "Max time difference: " << (this->globalMaxTimeDiff_ / 1000000) << " ms"
              << "\033[0m" << std::endl;
  }
}
/**
 * @brief Search closest PointStdDevStamped for a given frame
 *
 * @param[in] frame              - std::shared_ptr<OdometryFrame>:
 *                                 frame to search for
 * @return PointStdDevStamped    - PointStdDevStamped:
 *                                 closest PointStdDevStamped
 */
PointStdDevStamped KeyframeInterpolation::search_closest(
  const std::shared_ptr<OdometryFrame> & frame)
{
  auto closest_it = std::min_element(
    pos_frames_.begin(), pos_frames_.end(),
    [&frame](const PointStdDevStamped & a, const PointStdDevStamped & b) {
      return std::abs(frame->get_timestamp() - a.stamp) <
             std::abs(frame->get_timestamp() - b.stamp);
    });

  std::int64_t minDiff = std::abs(frame->get_timestamp() - closest_it->stamp);

  if (minDiff > this->globalMaxTimeDiff_) {
    this->globalMaxTimeDiff_ = minDiff;
  }

  return *closest_it;
}
/**
 * @brief Interpolate PointStdDevStamped for a given frame
 *
 * @param[in] frame              - std::shared_ptr<OdometryFrame>:
 *                                 frame to search for
 * @return PointStdDevStamped     - PointStdDevStamped:
 *                                 interpolated PointStdDevStamped
 */
PointStdDevStamped KeyframeInterpolation::interpolate_pos(
  const std::shared_ptr<OdometryFrame> & frame)
{
  const int numFrames = 2;

  // search closest pos timestamp smaller than keyframe timestamp
  auto lowerBound = std::lower_bound(
    this->pos_frames_.begin(), this->pos_frames_.end(), frame,
    [](const PointStdDevStamped & posFrame, const std::shared_ptr<OdometryFrame> & odomFrame) {
      return posFrame.stamp < odomFrame->get_timestamp();
    });
  size_t lowerIndex = (lowerBound == this->pos_frames_.begin())
                        ? 0
                        : std::distance(this->pos_frames_.begin(), lowerBound - 1);

  // Sanity check to check if enough pos frames before first keyframe
  if (
    lowerIndex < (numFrames - 1) || this->pos_frames_[lowerIndex].stamp > frame->get_timestamp()) {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\033[31mNot enough Pos frames provided for smallest LiDAR frame with timestamp "
              << static_cast<double>(frame->get_timestamp()) / 1000000000 << "\033[0m" << std::endl;
    exit(EXIT_FAILURE);
  }

  std::vector<int> selectedIndicesLow = {static_cast<int>(lowerIndex)};
  std::vector<int> selectedIndicesHigh = {static_cast<int>(lowerIndex)};

  // Search for PosFrames below lowerIndex with a euclidean distance of pos_delta_xyz
  for (int i = lowerIndex - 1; i >= 0 && selectedIndicesLow.size() < numFrames; --i) {
    if (
      i == 0 || this->pos_frames_[i].calc_dist(this->pos_frames_[selectedIndicesLow.back()]) >=
                  this->pos_delta_xyz_) {
      selectedIndicesLow.push_back(i);
    }
  }
  // Search for PosFrames above lowerIndex with a euclidean distance of pos_delta_xyz
  for (size_t i = lowerIndex + 1;
       i < this->pos_frames_.size() && selectedIndicesHigh.size() < (numFrames + 1); ++i) {
    if (
      i == this->pos_frames_.size() - 1 ||
      this->pos_frames_[i].calc_dist(this->pos_frames_[selectedIndicesHigh.back()]) >=
        this->pos_delta_xyz_) {
      selectedIndicesHigh.push_back(i);
    }
  }

  // Sanity check to check if enough pos frames found for interpolation
  if (selectedIndicesLow.size() < numFrames) {
    std::cout << "\033[31mNot enough Pos frames provided before the LiDAR frame with timestamp "
              << static_cast<double>(frame->get_timestamp()) / 1000000000 << "\033[0m" << std::endl;
    exit(EXIT_FAILURE);
  } else if (selectedIndicesHigh.size() < (numFrames + 1)) {
    std::cout << "\033[31mNot enough Pos frames provided after the LiDAR frame with timestamp "
              << static_cast<double>(frame->get_timestamp()) / 1000000000 << "\033[0m" << std::endl;
    exit(EXIT_FAILURE);
  }

  // Create set to delete double element
  std::set<int> uniqueSet;
  uniqueSet.insert(selectedIndicesLow.begin(), selectedIndicesLow.end());
  uniqueSet.insert(selectedIndicesHigh.begin(), selectedIndicesHigh.end());

  // This vector contains all indices of pos frames around our timestamp
  std::vector<int> resultVector(uniqueSet.begin(), uniqueSet.end());

  // Sort vector to make sure that alle indices are in right order
  std::sort(resultVector.begin(), resultVector.end());

  const int num_support_points = numFrames * 2;

  // Build up the knots along the spline. The first row is the nsecs offset from the
  // first stored position. The second, third and fourth rows are X, Y and Z respectively.
  Eigen::MatrixXd points(4, num_support_points);
  for (int i = 0; i < num_support_points; ++i) {
    // compute timestamps in reference to very first pos frame
    points(0, i) =
      std::abs(this->pos_frames_[resultVector[0]].stamp - this->pos_frames_[resultVector[i]].stamp);
    points(1, i) = this->pos_frames_[resultVector[i]].point.pos.x();
    points(2, i) = this->pos_frames_[resultVector[i]].point.pos.y();
    points(3, i) = this->pos_frames_[resultVector[i]].point.pos.z();
  }

  // The degree of the interpolating spline needs to be one less than the number of points
  // that are fitted to the spline.
  const auto fit =
    Eigen::SplineFitting<Eigen::Spline<double, 4>>::Interpolate(points, num_support_points - 1);
  Eigen::Spline<double, 4> spline(fit);

  // Normalize the timestamp of the frame we are interested in to [0,1]
  double divider = std::abs(this->pos_frames_[resultVector[0]].stamp - frame->get_timestamp()) /
                   (points.row(0).maxCoeff() - points.row(0).minCoeff());
  const Eigen::Vector4d values = spline(divider);

  // TODO(Maxi): check stddev values of surrounding pos frames and interpolate accordingly
  PointStdDev tmp_frame(values[1], values[2], values[3], 0.0, 0.0, 0.0);
  return PointStdDevStamped(tmp_frame, frame->get_timestamp());
}
/**
 * @brief Set parameters from config
 */
void KeyframeInterpolation::set_params(const std::string & config_path)
{
  // Load config
  YAML::Node config = YAML::LoadFile(config_path);

  this->odom_format_ = config["odom_format"].as<std::string>();
  this->stddev_threshold_ = config["stddev_threshold"].as<float>();
  this->keyframe_delta_x_ = config["keyframe_delta_x"].as<float>();
  this->keyframe_delta_angle_ = config["keyframe_delta_angle"].as<float>();
  this->downsample_resolution_ = config["downsample_resolution"].as<float>();
  this->interpolate_ = config["interpolate"].as<bool>();
  this->pos_delta_xyz_ = config["interp_pos_delta_xyz"].as<float>();
}
/**
 * @brief Visualize everything in rerun
 */
void KeyframeInterpolation::visualize()
{
  // Spawn a rerun stream
  this->rec_.spawn().exit_on_failure();

  // Visualize PosFrames
  viz_->pos2rerun(this->pos_frames_, this->rec_, "Orange", "pos_frames");
  viz_->pos2rerun(this->pos_keyframes_, this->rec_, "Green", "pos_keyframes");

  viz_->odom2rerun(this->frames_, this->rec_, "Orange", "odom_frames");
  viz_->odom2rerun(this->keyframes_, this->rec_, "Green", "odom_keyframes");
}
}  // namespace flexcloud
