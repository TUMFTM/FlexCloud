// Copyright 2024 Maximilian Leitenstern
#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <eigen3/unsupported/Eigen/Splines>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "file_io.hpp"
#include "utility.hpp"
namespace flexcloud
{
class KeyframeInterpolation
{
public:
  KeyframeInterpolation(
    const std::string & config_path, const std::string & pos_dir, const std::string & kitti_path,
    const std::string & pcd_dir, const std::string & dst_directory);
  std::vector<std::shared_ptr<OdometryFrame>> get_frames() const { return this->frames_; }
  std::vector<std::shared_ptr<OdometryFrame>> get_keyframes() const { return this->keyframes_; }
  std::vector<PosFrame> get_pos_frames() const { return this->pos_frames_; }
  std::vector<PosFrame> get_pos_keyframes() const { return this->pos_keyframes_; }

private:
  /**
   * @brief Load frames from a directory
   */
  void load(
    const std::string & pos_dir, const std::string & kitti_path, const std::string & pcd_dir);
  /**
   * @brief Save everything to directory
   */
  bool save(const std::string & dst_directory) const;
  /**
   * @brief Select keyframes
   */
  void select_keyframes(
    const float keyframe_delta_x, const float keyframe_delta_angle, const bool interpolate,
    const float pos_delta_xyz);
  PosFrame search_closest(const std::shared_ptr<OdometryFrame> & frame);
  PosFrame interpolate_pos(const std::shared_ptr<OdometryFrame> & frame, const float pos_delta_xyz);

private:
  std::shared_ptr<file_io> file_io_;
  std::vector<std::shared_ptr<OdometryFrame>> frames_;
  std::vector<PosFrame> pos_frames_;
  std::vector<std::shared_ptr<OdometryFrame>> keyframes_;
  std::vector<PosFrame> pos_keyframes_;
  bool interpolate_{false};
  std::int64_t globalMaxTimeDiff;
  float stddev_threshold_{1.0f};
  float downsample_resolution_{0.1f};
};
}  // namespace flexcloud
