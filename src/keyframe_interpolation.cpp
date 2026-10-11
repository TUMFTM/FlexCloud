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
#include <filesystem>  // NOLINT
#include <iostream>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "CLI/CLI.hpp"
#include "cli/cli_config.hpp"
namespace flexcloud
{
/// Heuristic detection of how to read the reference data at @p path.
enum class PositionsKind {
  TXT_FILE,  // single .txt file with one position per line
  TXT_DIR,   // directory of per-position .txt files
  BAG        // ROS 2 bag (.mcap / .db3 / .sqlite3 file, or a directory containing one)
};
bool is_bag_extension(const std::filesystem::path & ext)
{
  return ext == ".mcap" || ext == ".db3" || ext == ".sqlite3";
}
PositionsKind classify_positions_path(const std::string & p)
{
  std::filesystem::path path(p);
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("positions-path does not exist: " + p);
  }
  if (std::filesystem::is_regular_file(path)) {
    if (is_bag_extension(path.extension())) return PositionsKind::BAG;
    if (path.extension() == ".txt") return PositionsKind::TXT_FILE;
    throw std::runtime_error("Unsupported positions-path file extension: " + p);
  }
  if (std::filesystem::is_directory(path)) {
    for (const auto & entry : std::filesystem::directory_iterator(path)) {
      if (entry.is_regular_file() && is_bag_extension(entry.path().extension())) {
        return PositionsKind::BAG;
      }
    }
    return PositionsKind::TXT_DIR;
  }
  throw std::runtime_error("positions-path is neither a regular file nor a directory: " + p);
}
KeyframeInterpolation::KeyframeInterpolation(const config::KeyframeInterpolationConfig & cli_cfg)
{
  // Load
  load(cli_cfg);

  // Interpolate keyframes
  select_keyframes(cli_cfg);

  // Save keyframes
  if (!save(cli_cfg.out_dir)) {
    throw std::runtime_error("Could not write the keyframes to '" + cli_cfg.out_dir + "'");
  }
}
/**
 * @brief Load frames based on CLI configuration.
 */
void KeyframeInterpolation::load(const config::KeyframeInterpolationConfig & cli_cfg)
{
  // Load reference positions — auto-detect reader from positions_path
  this->positions_.clear();
  const PositionsKind kind = classify_positions_path(cli_cfg.positions_path);
  std::optional<Eigen::Vector3d> origin = std::nullopt;
  if (cli_cfg.origin.size() == 3) {
    origin = Eigen::Vector3d(cli_cfg.origin[0], cli_cfg.origin[1], cli_cfg.origin[2]);
  }
  switch (kind) {
    case PositionsKind::TXT_FILE:
      this->positions_ =
        file_io_->load_positions_file(cli_cfg.positions_path, cli_cfg.stddev_threshold, origin);
      break;
    case PositionsKind::TXT_DIR:
      this->positions_ =
        file_io_->load_positions_dir(cli_cfg.positions_path, cli_cfg.stddev_threshold, origin);
      break;
    case PositionsKind::BAG: {
      if (cli_cfg.pos_topic.empty()) {
        throw std::runtime_error("--pos-topic is required when positions-path is a ROS 2 bag");
      }
      rosbag_io io(
        cli_cfg.positions_path, cli_cfg.pos_topic, cli_cfg.target_frame, cli_cfg.stddev_threshold,
        origin);
      this->positions_ = io.run();
      break;
    }
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
  if (this->positions_.empty()) {
    throw std::runtime_error("No reference positions to match the keyframes with");
  }
  if (this->poses_.empty()) {
    throw std::runtime_error("No SLAM poses loaded from '" + cli_cfg.poses_path + "'");
  }

  this->key_poses_.clear();
  this->key_positions_.clear();
  std::size_t skipped = 0;

  // Set keyframes; the selection only depends on the SLAM motion, so it compares against the
  // last selected keyframe, whether a reference position was found for it or not
  const PoseStamped * last_keyframe = nullptr;
  for (const auto & frame : this->poses_) {
    if (last_keyframe != nullptr) {
      const Eigen::Isometry3d delta = last_keyframe->pose.pose.inverse() * frame.pose.pose;
      const double delta_x = delta.translation().norm();
      const double delta_angle = Eigen::AngleAxisd(delta.linear()).angle();
      if (delta_x <= cli_cfg.keyframe_delta_x && delta_angle <= cli_cfg.keyframe_delta_angle) {
        continue;
      }
    }
    last_keyframe = &frame;

    // Keyframes outside the reference recording or inside a gap of it get no position
    const std::optional<PointStdDevStamped> position =
      cli_cfg.interpolate
        ? interpolate_pos(frame, cli_cfg.interp_pos_delta_xyz, cli_cfg.max_time_diff)
        : search_closest(frame, cli_cfg.max_time_diff);
    if (!position) {
      ++skipped;
      continue;
    }
    this->key_poses_.push_back(frame);
    this->key_positions_.push_back(*position);
  }

  if (skipped > 0) {
    std::cout << "\033[33m!! Skipped " << skipped << " keyframes without reference positions "
              << "within " << cli_cfg.max_time_diff << " s !!\033[0m" << std::endl;
  }
  if (this->key_poses_.empty()) {
    throw std::runtime_error(
      "No keyframe has reference positions within --max-time-diff; do the SLAM poses and the "
      "reference positions cover the same time span?");
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
 * @param[in] max_time_diff      - double:
 *                                maximum time difference [s] to the closest position
 * @return                       - std::optional<PointStdDevStamped>:
 *                                 closest position, std::nullopt if none within max_time_diff
 */
std::optional<PointStdDevStamped> KeyframeInterpolation::search_closest(
  const PoseStamped & pose, const double max_time_diff)
{
  auto closest_it = std::min_element(
    positions_.begin(), positions_.end(),
    [&pose](const PointStdDevStamped & a, const PointStdDevStamped & b) {
      return std::abs(pose.stamp - a.stamp) < std::abs(pose.stamp - b.stamp);
    });

  std::int64_t minDiff = std::abs(pose.stamp - closest_it->stamp);
  if (minDiff > static_cast<std::int64_t>(max_time_diff * 1.0e9)) {
    return std::nullopt;
  }

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
 * @param[in] max_time_diff       - double:
 *                                maximum time [s] to the nearest position before and after it
 * @return                        - std::optional<PointStdDevStamped>:
 *                                 interpolated position, std::nullopt if the frame is not
 *                                 covered by the reference positions
 */
std::optional<PointStdDevStamped> KeyframeInterpolation::interpolate_pos(
  const PoseStamped & pose, const double pos_delta_xyz, const double max_time_diff)
{
  const int numFrames = 2;
  const auto max_dt = static_cast<std::int64_t>(max_time_diff * 1.0e9);

  // search closest pos timestamp smaller than keyframe timestamp
  auto lowerBound = std::lower_bound(
    this->positions_.begin(), this->positions_.end(), pose,
    [](const PointStdDevStamped & position, const PoseStamped & current_pose) {
      return position.stamp < current_pose.stamp;
    });
  // Positions are needed close in time on both sides, otherwise the frame lies outside the
  // reference recording or inside a gap of it
  if (
    lowerBound == this->positions_.begin() || lowerBound == this->positions_.end() ||
    pose.stamp - std::prev(lowerBound)->stamp > max_dt || lowerBound->stamp - pose.stamp > max_dt) {
    return std::nullopt;
  }
  size_t lowerIndex = std::distance(this->positions_.begin(), lowerBound - 1);

  if (lowerIndex < (numFrames - 1)) {
    return std::nullopt;
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
  if (selectedIndicesLow.size() < numFrames || selectedIndicesHigh.size() < (numFrames + 1)) {
    return std::nullopt;
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
  // Spawn a rerun stream; the outputs are already written, so a missing viewer is not an error
  const rerun::Error spawned = this->rec_.spawn();
  if (spawned.is_err()) {
    std::cout << "\033[33m!! Could not spawn Rerun viewer, skipping visualization: "
              << spawned.description << " !!\033[0m" << std::endl;
    return;
  }

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
    "\nExamples (positions-path auto-detected):\n"
    "  # single txt file with one position per line\n"
    "  flexcloud-keyframe-interpolation positions.txt poses_kitti.txt\n\n"
    "  # directory of per-position txt files\n"
    "  flexcloud-keyframe-interpolation /path/to/positions/ poses_kitti.txt\n\n"
    "  # ROS 2 bag with NavSatFix on /sensor/gnss/fix\n"
    "  flexcloud-keyframe-interpolation /path/to/bag.mcap poses_kitti.txt /path/to/out \\\n"
    "      --pos-topic /sensor/gnss/fix --origin 48.262 11.667 0.0 --target-frame base_link\n\n"
    "  # ROS 2 bag with Odometry, moved into the SLAM pose frame via TF\n"
    "  flexcloud-keyframe-interpolation /path/to/bag.mcap poses_kitti.txt /path/to/out \\\n"
    "      --pos-topic /odom --target-frame base_link\n");

  CLI11_PARSE(app, argc, argv);

  try {
    flexcloud::KeyframeInterpolation set_frames(cfg);
    set_frames.visualize();
  } catch (const std::exception & e) {
    std::cerr << "\033[1;31m!! " << e.what() << " !!\033[0m" << std::endl;
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return 0;
}
