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
KeyframeInterpolation::KeyframeInterpolation(
  const std::string & config_path, const std::string & pos_dir, const std::string & odom_path,
  const std::string & dst_directory)
{
  this->global_time_diff_ = 0.0f;
  // Load config
  YAML::Node config = YAML::LoadFile(config_path);

  this->stddev_threshold_ = config["stddev_threshold"].as<float>();
  this->keyframe_delta_x_ = config["keyframe_delta_x"].as<float>();
  this->keyframe_delta_angle_ = config["keyframe_delta_angle"].as<float>();
  this->interpolate_ = config["interpolate"].as<bool>();
  this->pos_delta_xyz_ = config["interp_pos_delta_xyz"].as<float>();

  // Load
  load(pos_dir, odom_path);

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
 * * @param[in] pcd_dir             - std::string:
 * *                                 absolute path to directory
 */
void KeyframeInterpolation::load(const std::string & pos_dir, const std::string & odom_path)
{
  // Load position frames
  this->positions_.clear();
  this->positions_ = file_io_->load_positions_dir(pos_dir, this->stddev_threshold_);

  // Load odometry frames
  this->poses_.clear();
  // Load kitti odometry file
  std::cout << "Loading odometry poses from " << odom_path << std::endl;
  this->poses_ = file_io_->load_poses(odom_path);
  std::cout << "Loaded " << this->poses_.size() << " odometry frames" << std::endl;
}
/**
 * @brief Save everything to directory
 *
 * @param[in] dst_directory       - std::string:
 *                                 absolute path to directory
 */
bool KeyframeInterpolation::save(const std::string & dst_directory) const
{
  if (this->key_poses_.empty()) {
    return false;
  }

  // Create directory
  if (!boost::filesystem::is_directory(dst_directory)) {
    boost::filesystem::create_directories(dst_directory);
  }

  if (!file_io_->save_poses(dst_directory + "/poses_keyframes.txt", this->key_poses_)) {
    return false;
  }

  if (!file_io_->save_positions(
        dst_directory + "/positions_interpolated.txt", this->key_positions_)) {
    return false;
  }
  return true;
}
/**
 * @brief Select keyframes
 */
void KeyframeInterpolation::select_keyframes()
{
  if (this->positions_.empty() || this->poses_.empty()) {
    return;
  }

  this->key_poses_.clear();
  this->key_positions_.clear();

  // Handle first frame
  this->key_poses_.push_back(this->poses_.front());
  if (this->interpolate_) {
    /* Interpolate position of frame */
    this->key_positions_.push_back(interpolate_pos(this->poses_.front()));
  } else {
    /* Search GPS frame for first frame */
    this->key_positions_.push_back(search_closest(this->poses_.front()));
  }

  // Set keyframes
  for (const auto & frame : this->poses_) {
    const auto & last_keyframe_pose = this->key_poses_.back().pose;
    const auto & current_frame_pose = frame.pose.pose;

    Eigen::Isometry3d delta = last_keyframe_pose.pose.inverse() * current_frame_pose;
    double delta_x = delta.translation().norm();
    double delta_angle = Eigen::AngleAxisd(delta.linear()).angle();

    if (delta_x > this->keyframe_delta_x_ || delta_angle > this->keyframe_delta_angle_) {
      this->key_poses_.push_back(frame);

      if (this->interpolate_) {
        // Interpolate position of frame
        this->key_positions_.push_back(interpolate_pos(frame));
      } else {
        // search for corresponding pos file that is closest (time) to frame
        this->key_positions_.push_back(search_closest(frame));
      }
    }
  }

  if (!this->interpolate_) {
    // Output max time difference that occured during keyframe selection
    std::cout << "\033[1;32m"
              << "Max time difference: " << (this->global_time_diff_ / 1000000) << " ms"
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
  const PoseStamped & pose)
{
  auto closest_it = std::min_element(
    positions_.begin(), positions_.end(),
    [&pose](const PointStdDevStamped & a, const PointStdDevStamped & b) {
      return std::abs(pose.stamp - a.stamp) <
             std::abs(pose.stamp - b.stamp);
    });

  std::int64_t minDiff = std::abs(pose.stamp - closest_it->stamp);

  if (minDiff > this->global_time_diff_) {
    this->global_time_diff_ = minDiff;
  }

  return *closest_it;
}
/**
 * @brief Interpolate PointStdDevStamped for a given pose
 *
 * @param[in] frame              - std::shared_ptr<OdometryFrame>:
 *                                 frame to search for
 * @return PointStdDevStamped     - PointStdDevStamped:
 *                                 interpolated PointStdDevStamped
 */
PointStdDevStamped KeyframeInterpolation::interpolate_pos(
  const PoseStamped & pose)
{
  const int numFrames = 2;

  // search closest pos timestamp smaller than keyframe timestamp
  auto lowerBound = std::lower_bound(
    this->positions_.begin(), this->positions_.end(), pose,
    [](const PointStdDevStamped & position, const PoseStamped & current_pose) {
      return position.stamp < current_pose.stamp;
    });
  size_t lowerIndex = (lowerBound == this->positions_.begin())
                        ? 0
                        : std::distance(this->positions_.begin(), lowerBound - 1);

  // Sanity check to check if enough pos frames before first keyframe
  if (
    lowerIndex < (numFrames - 1) || this->positions_[lowerIndex].stamp > pose.stamp) {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\033[31mNot enough Pos frames provided for smallest LiDAR frame with timestamp "
              << static_cast<double>(pose.stamp) / 1000000000 << "\033[0m" << std::endl;
    exit(EXIT_FAILURE);
  }

  std::vector<int> selectedIndicesLow = {static_cast<int>(lowerIndex)};
  std::vector<int> selectedIndicesHigh = {static_cast<int>(lowerIndex)};

  // Search for PosFrames below lowerIndex with a euclidean distance of pos_delta_xyz
  for (int i = lowerIndex - 1; i >= 0 && selectedIndicesLow.size() < numFrames; --i) {
    if (
      i == 0 || this->positions_[i].calc_dist(this->positions_[selectedIndicesLow.back()]) >=
                  this->pos_delta_xyz_) {
      selectedIndicesLow.push_back(i);
    }
  }
  // Search for PosFrames above lowerIndex with a euclidean distance of pos_delta_xyz
  for (size_t i = lowerIndex + 1;
       i < this->positions_.size() && selectedIndicesHigh.size() < (numFrames + 1); ++i) {
    if (
      i == this->positions_.size() - 1 ||
      this->positions_[i].calc_dist(this->positions_[selectedIndicesHigh.back()]) >=
        this->pos_delta_xyz_) {
      selectedIndicesHigh.push_back(i);
    }
  }

  // Sanity check to check if enough pos frames found for interpolation
  if (selectedIndicesLow.size() < numFrames) {
    std::cout << "\033[31mNot enough Pos frames provided before the LiDAR frame with timestamp "
              << static_cast<double>(pose.stamp) / 1000000000 << "\033[0m" << std::endl;
    exit(EXIT_FAILURE);
  } else if (selectedIndicesHigh.size() < (numFrames + 1)) {
    std::cout << "\033[31mNot enough Pos frames provided after the LiDAR frame with timestamp "
              << static_cast<double>(pose.stamp) / 1000000000 << "\033[0m" << std::endl;
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
      std::abs(this->positions_[resultVector[0]].stamp - this->positions_[resultVector[i]].stamp);
    points(1, i) = this->positions_[resultVector[i]].point.pos.x();
    points(2, i) = this->positions_[resultVector[i]].point.pos.y();
    points(3, i) = this->positions_[resultVector[i]].point.pos.z();
  }

  // The degree of the interpolating spline needs to be one less than the number of points
  // that are fitted to the spline.
  const auto fit =
    Eigen::SplineFitting<Eigen::Spline<double, 4>>::Interpolate(points, num_support_points - 1);
  Eigen::Spline<double, 4> spline(fit);

  // Normalize the timestamp of the frame we are interested in to [0,1]
  double divider = std::abs(this->positions_[resultVector[0]].stamp - pose.stamp) /
                   (points.row(0).maxCoeff() - points.row(0).minCoeff());
  const Eigen::Vector4d values = spline(divider);

  // TODO(Maxi): check stddev values of surrounding pos frames and interpolate accordingly
  PointStdDev tmp_frame(values[1], values[2], values[3], 0.0, 0.0, 0.0);
  return PointStdDevStamped(tmp_frame, pose.stamp);
}
void KeyframeInterpolation::visualize()
{
  // Spawn a rerun stream
  this->rec_.spawn().exit_on_failure();

  // Visualize PosFrames
  viz_->pos2rerun(this->positions_, this->rec_, "Orange", "positions");
  viz_->pos2rerun(this->key_positions_, this->rec_, "Green", "positions_inter");

  viz_->pose2rerun(this->poses_, this->rec_, "Orange", "poses");
  viz_->pose2rerun(this->key_poses_, this->rec_, "Green", "poses_key");
}
}  // namespace flexcloud
/**
 * @brief initialize package
 */
int main(int argc, char * argv[])
{
  // Check the number of arguments
  if (argc < 5) {
    // Tell the user how to run the program
    std::cerr << "Usage: " << argv[0]
              << " <config_path> <pos_dir_path> <kitti_odom_path> <dst_dir_path>"
              << std::endl;
    return 1;
  }
  if (argc == 5) {
    // Only interpolation without accumulating clouds
    flexcloud::KeyframeInterpolation set_frames(argv[1], argv[2], argv[3], argv[4]);
    set_frames.visualize();
  } else {
    std::cerr << "Too many arguments" << std::endl;
    return 1;
  }
  return 0;
}
