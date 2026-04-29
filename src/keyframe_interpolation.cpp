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
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <string>
#include <vector>

#include "CLI/CLI.hpp"
#include "cli/cli_config.hpp"
namespace flexcloud
{
KeyframeInterpolation::KeyframeInterpolation(const config::KeyframeInterpolationConfig & cli_cfg)
{
  // Load
  load(cli_cfg);

  // Interpolate keyframes
  select_keyframes(cli_cfg);

  // Save keyframes
  save(cli_cfg.out_dir);
}
/**
 * @brief Load frames based on CLI configuration.
 */
void KeyframeInterpolation::load(const config::KeyframeInterpolationConfig & cli_cfg)
{
  // Load reference positions
  this->positions_.clear();
  if (cli_cfg.use_bag()) {
    std::optional<Eigen::Vector3d> origin;
    if (cli_cfg.origin.size() == 3) {
      origin = Eigen::Vector3d(cli_cfg.origin[0], cli_cfg.origin[1], cli_cfg.origin[2]);
    }
    rosbag_io io(
      cli_cfg.pos_bag, cli_cfg.pos_topic, cli_cfg.target_frame, cli_cfg.stddev_threshold, origin);
    this->positions_ = io.run();
  } else {
    this->positions_ = file_io_->load_positions_dir(cli_cfg.pos_dir, cli_cfg.stddev_threshold);
  }

  // Load SLAM poses (GLIM format)
  this->poses_.clear();
  std::cout << "Loading odometry poses from " << cli_cfg.poses_path << std::endl;
  this->poses_ = file_io_->load_poses(cli_cfg.poses_path);
  std::cout << "Loaded " << this->poses_.size() << " odometry frames" << std::endl;
}
/**
 * @brief Save everything to directory
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
void KeyframeInterpolation::select_keyframes(const config::KeyframeInterpolationConfig & cli_cfg)
{
  if (this->positions_.empty() || this->poses_.empty()) {
    return;
  }

  this->key_poses_.clear();
  this->key_positions_.clear();

  // Handle first frame
  this->key_poses_.push_back(this->poses_.front());
  if (cli_cfg.interpolate) {
    // Interpolate position of frame
    this->key_positions_.push_back(interpolate_pos(this->poses_.front(), cli_cfg.interp_pos_delta_xyz));
  } else {
    // Search GPS frame for first frame
    this->key_positions_.push_back(search_closest(this->poses_.front()));
  }

  // Set keyframes
  for (const auto & frame : this->poses_) {
    const auto & last_keyframe_pose = this->key_poses_.back().pose;
    const auto & current_frame_pose = frame.pose.pose;

    Eigen::Isometry3d delta = last_keyframe_pose.pose.inverse() * current_frame_pose;
    double delta_x = delta.translation().norm();
    double delta_angle = Eigen::AngleAxisd(delta.linear()).angle();

    if (delta_x > cli_cfg.keyframe_delta_x || delta_angle > cli_cfg.keyframe_delta_angle) {
      this->key_poses_.push_back(frame);

      if (cli_cfg.interpolate) {
        // Interpolate position of frame
        this->key_positions_.push_back(interpolate_pos(frame, cli_cfg.interp_pos_delta_xyz));
      } else {
        // search for corresponding pos file that is closest (time) to frame
        this->key_positions_.push_back(search_closest(frame));
      }
    }
  }

  if (!cli_cfg.interpolate) {
    // Output max time difference that occured during keyframe selection
    std::cout << "\033[1;32m"
              << "Max time difference: " << (this->max_time_diff_ / 1000000) << " ms"
              << "\033[0m" << std::endl;
  }
}
/**
 * @brief Search closest PointStdDevStamped for a given frame
 *
 * @param[in] pose               - PoseStamped:
 *                                frame to search for
 * @return PointStdDevStamped    - PointStdDevStamped:
 *                                 closest PointStdDevStamped
 */
PointStdDevStamped KeyframeInterpolation::search_closest(const PoseStamped & pose)
{
  auto closest_it = std::min_element(
    positions_.begin(), positions_.end(),
    [&pose](const PointStdDevStamped & a, const PointStdDevStamped & b) {
      return std::abs(pose.stamp - a.stamp) < std::abs(pose.stamp - b.stamp);
    });

  std::int64_t minDiff = std::abs(pose.stamp - closest_it->stamp);

  if (minDiff > this->max_time_diff_) {
    this->max_time_diff_ = minDiff;
  }

  return *closest_it;
}
/**
 * @brief Interpolate PointStdDevStamped for a given pose
 *
 * @param[in] pose                - PoseStamped:
 *                                frame to interpolate for
 * @return PointStdDevStamped     - PointStdDevStamped:
 *                                 interpolated PointStdDevStamped
 */
PointStdDevStamped KeyframeInterpolation::interpolate_pos(
  const PoseStamped & pose, const double pos_delta_xyz)
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

  if (lowerIndex < (numFrames - 1) || this->positions_[lowerIndex].stamp > pose.stamp) {
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
      i == 0 ||
      this->positions_[i].calc_dist(this->positions_[selectedIndicesLow.back()]) >= pos_delta_xyz) {
      selectedIndicesLow.push_back(i);
    }
  }
  // Search for PosFrames above lowerIndex with a euclidean distance of pos_delta_xyz
  for (size_t i = lowerIndex + 1;
       i < this->positions_.size() && selectedIndicesHigh.size() < (numFrames + 1); ++i) {
    if (
      i == this->positions_.size() - 1 ||
      this->positions_[i].calc_dist(this->positions_[selectedIndicesHigh.back()]) >=
        pos_delta_xyz) {
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
  rclcpp::init(argc, argv);

  CLI::App app{
    "Select keyframes from a SLAM trajectory and interpolate the corresponding "
    "reference positions, either from per-position txt files or from a ROS 2 bag "
    "(NavSatFix or Odometry messages)."};
  app.name("keyframe_interpolation");

  flexcloud::config::KeyframeInterpolationConfig cfg;
  cfg.add_cli_options(&app);

  app.footer(
    "\nExamples:\n"
    "  # txt directory of per-position files (output to current directory)\n"
    "  ros2 run flexcloud keyframe_interpolation /path/to/poses_kitti.txt \\\n"
    "      --pos-dir /path/to/positions/\n\n"
    "  # ROS 2 bag with NavSatFix on /sensor/gnss/fix\n"
    "  ros2 run flexcloud keyframe_interpolation /path/to/poses_kitti.txt /path/to/out \\\n"
    "      --pos-bag /path/to/bag.mcap --pos-topic /sensor/gnss/fix \\\n"
    "      --target-frame base_link\n");

  CLI11_PARSE(app, argc, argv);

  if (cfg.pos_dir.empty() && cfg.pos_bag.empty()) {
    std::cerr << "Either --pos-dir or --pos-bag must be provided." << std::endl;
    return 1;
  }
  if (!cfg.pos_bag.empty() && cfg.pos_topic.empty()) {
    std::cerr << "--pos-topic is required when --pos-bag is set." << std::endl;
    return 1;
  }

  flexcloud::KeyframeInterpolation set_frames(cfg);
  set_frames.visualize();

  rclcpp::shutdown();
  return 0;
}
