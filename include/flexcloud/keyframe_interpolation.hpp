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
   * 
   * @param[in] pos_dir             - std::string:
   *                                  absolute path to directory
   * @param[in] kitti_path          - std::string:
   *                                  path to kitti odometry
   * @param[in] pcd_dir             - std::string:
   *                                  absolute path to directory
   */
  void load(
    const std::string & pos_dir, const std::string & kitti_path, const std::string & pcd_dir);
  /**
   * @brief Save everything to directory
   * 
   * @param[in] dst_directory       - std::string:
   *                                  absolute path to directory
   */
  bool save(const std::string & dst_directory) const;
  /**
   * @brief Select keyframes
   * 
   * @param[in] keyframe_delta_x     - float:
   *                                  delta x for keyframe selection
   * @param[in] keyframe_delta_angle - float:
   *                                  delta angle for keyframe selection
   * @param[in] interpolate          - bool:
   *                                  interpolate keyframes
   * @param[in] pos_delta_xyz        - float:
   *                                  delta xyz for keyframe selection
   */
  void select_keyframes(
    const float keyframe_delta_x, const float keyframe_delta_angle, const bool interpolate,
    const float pos_delta_xyz);
  /**
   * @brief Search closest PosFrame for a given frame
   * 
   * @param[in] frame              - std::shared_ptr<OdometryFrame>:
   *                                 frame to search for
   * @return PosFrame             - PosFrame:
   *                                 closest PosFrame
   */
  PosFrame search_closest(const std::shared_ptr<OdometryFrame> & frame);
  /**
   * @brief Interpolate PosFrame for a given frame
   * 
   * @param[in] frame              - std::shared_ptr<OdometryFrame>:
   *                                 frame to interpolate
   * @param[in] pos_delta_xyz      - float:
   *                                 delta xyz for interpolation
   * @return PosFrame             - PosFrame:
   *                                 interpolated PosFrame
   */
  PosFrame interpolate_pos(const std::shared_ptr<OdometryFrame> & frame, const float pos_delta_xyz);

private:
  // File IO
  std::shared_ptr<file_io> file_io_;

  // Data
  std::vector<std::shared_ptr<OdometryFrame>> frames_;
  std::vector<PosFrame> pos_frames_;
  std::vector<std::shared_ptr<OdometryFrame>> keyframes_;
  std::vector<PosFrame> pos_keyframes_;

  // Config parameters
  bool interpolate_{false};
  std::int64_t globalMaxTimeDiff;
  float stddev_threshold_{1.0f};
  float downsample_resolution_{0.1f};
};
}  // namespace flexcloud
