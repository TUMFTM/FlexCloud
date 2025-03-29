// Copyright 2024 Maximilian Leitenstern
// based on: https://github.com/koide3/interactive_slam/blob/master/src/odometry2graph.cpp

#include "keyframe_interpolation.hpp"

#include "yaml-cpp/yaml.h"
namespace flexcloud
{
KeyframeInterpolation::KeyframeInterpolation(
  const std::string & config_path, const std::string & pos_dir, const std::string & kitti_path,
  const std::string & pcd_dir, const std::string & dst_directory)
{
  this->globalMaxTimeDiff = 0.0f;
  // Load config
  YAML::Node config = YAML::LoadFile(config_path);

  this->stddev_threshold_ = config["stddev_threshold"].as<float>();
  const float keyframe_delta_x = config["keyframe_delta_x"].as<float>();
  const float keyframe_delta_angle = config["keyframe_delta_angle"].as<float>();
  this->downsample_resolution_ = config["downsample_resolution"].as<float>();
  this->interpolate_ = config["interpolate"].as<bool>();
  const float pos_delta_xyz = config["interp_pos_delta_xyz"].as<float>();

  // Load position frames
  load(pos_dir, kitti_path, pcd_dir);

  // Interpolate keyframes
  select_keyframes(keyframe_delta_x, keyframe_delta_angle, this->interpolate_, pos_delta_xyz);

  // Save keyframes
  save(dst_directory);
}
/**
 * @brief Load frames from a directory
 */
void KeyframeInterpolation::load(
  const std::string & pos_dir, const std::string & kitti_path, const std::string & pcd_dir)
{
  // Load position frames
  this->pos_frames_.clear();
  this->pos_frames_ = file_io_->load_pos_frames(pos_dir, this->stddev_threshold_);

  // Load odometry frames
  this->frames_.clear();
  // Load kitti odometry file
  std::cout << "Loading odometry poses from " << kitti_path << std::endl;
  std::vector<Eigen::Isometry3d> poses = file_io_->load_kitti_odom(kitti_path);
  // Load pcd cloud filenames
  std::cout << "Loading pcd-clouds from " << pcd_dir << std::endl;
  std::vector<std::string> pcd_filenames = file_io_->load_clouds(pcd_dir);
  // Check if sizes match
  if (pcd_filenames.size() != poses.size()) {
    std::cerr << "Number of pcd files and poses do not match: " << pcd_filenames.size() << " vs "
              << poses.size() << std::endl;
    return;
  }

  // Load pcd files and creates frames
  for (size_t i = 0; i < pcd_filenames.size(); ++i) {
    auto frame = OdometryFrame::load(pcd_filenames[i], poses[i], this->downsample_resolution_);
    if (frame == nullptr) {
      continue;
    }
    this->frames_.push_back(frame);
  }
  std::cout << "Loaded " << this->frames_.size() << " odometry frames" << std::endl;
}
bool KeyframeInterpolation::save(const std::string & dst_directory) const
{
  if (this->keyframes_.empty()) {
    return false;
  }

  // Create directory
  if (!boost::filesystem::is_directory(dst_directory)) {
    boost::filesystem::create_directories(dst_directory);
  }

  // Save files
  if (!file_io_->save_graph(dst_directory + "/graph.g2o", this->keyframes_)) {
    return false;
  }

  if (!file_io_->save_kitti(dst_directory + "/kitti_poses.txt", this->keyframes_)) {
    return false;
  }

  if (!file_io_->save_keyframes(dst_directory, this->keyframes_, this->downsample_resolution_)) {
    return false;
  }

  if (!file_io_->save_pos_frames(dst_directory + "/poseData.txt", this->pos_keyframes_)) {
    return false;
  }
  std::cout << "Everything saved successfully" << std::endl;
  return true;
}
void KeyframeInterpolation::select_keyframes(
  float keyframe_delta_x, float keyframe_delta_angle, bool interpolate, float pos_delta_xyz)
{
  if (this->frames_.empty() || this->pos_frames_.empty()) {
    return;
  }

  this->keyframes_.clear();
  this->pos_keyframes_.clear();

  // Handle first frame
  this->keyframes_.push_back(this->frames_.front());
  if (interpolate) {
    /* Interpolate position of frame */
    this->pos_keyframes_.push_back(interpolate_pos(this->frames_.front(), pos_delta_xyz));
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

    if (delta_x > keyframe_delta_x || delta_angle > keyframe_delta_angle) {
      this->keyframes_.push_back(frame);

      if (interpolate) {
        // Interpolate position of frame
        this->pos_keyframes_.push_back(interpolate_pos(frame, pos_delta_xyz));
      } else {
        // search for corresponding pos file that is closest (time) to frame
        this->pos_keyframes_.push_back(search_closest(frame));
      }
    }
  }

  if (!interpolate) {
    // Output max time difference that occured during keyframe selection
    std::cout << "\033[1;32m"
              << "Max time difference: " << (globalMaxTimeDiff / 1000000) << " ms"
              << "\033[0m" << std::endl;
  }
}
PosFrame KeyframeInterpolation::search_closest(const std::shared_ptr<OdometryFrame> & frame)
{
  auto closest_it = std::min_element(
    pos_frames_.begin(), pos_frames_.end(), [&frame](const PosFrame & a, const PosFrame & b) {
      return std::abs(frame->get_timestamp() - a.get_timestamp()) <
             std::abs(frame->get_timestamp() - b.get_timestamp());
    });

  std::int64_t minDiff = std::abs(frame->get_timestamp() - closest_it->get_timestamp());

  if (minDiff > globalMaxTimeDiff) {
    globalMaxTimeDiff = minDiff;
  }

  return *closest_it;
}
PosFrame KeyframeInterpolation::interpolate_pos(
  const std::shared_ptr<OdometryFrame> & frame, const float pos_delta_xyz)
{
  const int numFrames = 2;

  // search closest pos timestamp smaller than keyframe timestamp
  auto lowerBound = std::lower_bound(
    this->pos_frames_.begin(), this->pos_frames_.end(), frame,
    [](const PosFrame & posFrame, const std::shared_ptr<OdometryFrame> & odomFrame) {
      return posFrame.get_timestamp() < odomFrame->get_timestamp();
    });
  size_t lowerIndex = (lowerBound == this->pos_frames_.begin())
                        ? 0
                        : std::distance(this->pos_frames_.begin(), lowerBound - 1);

  // Sanity check to check if enough pos frames before first keyframe
  if (
    lowerIndex < (numFrames - 1) ||
    this->pos_frames_[lowerIndex].get_timestamp() > frame->get_timestamp()) {
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
                  pos_delta_xyz) {
      selectedIndicesLow.push_back(i);
    }
  }
  // Search for PosFrames above lowerIndex with a euclidean distance of pos_delta_xyz
  for (size_t i = lowerIndex + 1;
       i < this->pos_frames_.size() && selectedIndicesHigh.size() < (numFrames + 1); ++i) {
    if (
      i == this->pos_frames_.size() - 1 ||
      this->pos_frames_[i].calc_dist(this->pos_frames_[selectedIndicesHigh.back()]) >=
        pos_delta_xyz) {
      selectedIndicesHigh.push_back(i);
    }
  }

  // Sanity check to check if enough pos frames found for interpolation
  if (selectedIndicesLow.size() < numFrames) {
    std::cout
      << "\033[31mNot enough Pos frames provided before the LiDAR frame with timestamp "
      << static_cast<double>(frame->get_timestamp()) / 1000000000 << "\033[0m" << std::endl;
    exit(EXIT_FAILURE);
  } else if (selectedIndicesHigh.size() < (numFrames + 1)) {
    std::cout
      << "\033[31mNot enough Pos frames provided after the LiDAR frame with timestamp "
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
    points(0, i) = std::abs(
      this->pos_frames_[resultVector[0]].get_timestamp() -
      this->pos_frames_[resultVector[i]].get_timestamp());
    points(1, i) = this->pos_frames_[resultVector[i]].x_pos;
    points(2, i) = this->pos_frames_[resultVector[i]].y_pos;
    points(3, i) = this->pos_frames_[resultVector[i]].z_pos;
  }

  // The degree of the interpolating spline needs to be one less than the number of points
  // that are fitted to the spline.
  const auto fit =
    Eigen::SplineFitting<Eigen::Spline<double, 4>>::Interpolate(points, num_support_points - 1);
  Eigen::Spline<double, 4> spline(fit);

  // Normalize the timestamp of the frame we are interested in to [0,1]
  double divider =
    std::abs(this->pos_frames_[resultVector[0]].get_timestamp() - frame->get_timestamp()) /
    (points.row(0).maxCoeff() - points.row(0).minCoeff());
  const Eigen::Vector4d values = spline(divider);

  PosFrame tmp_frame(
    frame->stamp_sec, frame->stamp_nsec, values[1], values[2], values[3], 0.0, 0.0, 0.0);

  return tmp_frame;
}
}  // namespace flexcloud
