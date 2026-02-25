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

#pragma once
//

#include <Eigen/Geometry>
#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
namespace flexcloud
{
/**
 * @brief struct to represent metric position with standard deviation
 *
 * @param[in] x                     - double:
 *                                    x-position
 * @param[in] y                     - double:
 *                                    y-position
 * @param[in] z                     - double:
 *                                    z-position
 * @param[in] x_stddev              - double:
 *                                    x standard deviation
 * @param[in] y_stddev              - double:
 *                                    y standard deviation
 * @param[in] z_stddev              - double:
 *                                    z standard deviation
 */
struct PointStdDev
{
public:
  PointStdDev(
    const double x, const double y, const double z, const double x_stddev, const double y_stddev,
    const double z_stddev)
  : pos(x, y, z), stddev(x_stddev, y_stddev, z_stddev)
  {
  }

public:
  Eigen::Vector3d pos;
  Eigen::Vector3d stddev;
};
/**
 * @brief PointStdDev with timestamp
 *
 * @param [in] PointStdDev point with stddev
 *
 * @param [in] stamp timestamp in nanoseconds
 */
struct PointStdDevStamped
{
public:
  PointStdDevStamped(const PointStdDev & point, const std::int64_t stamp)
  : point(point), stamp(stamp)
  {
  }
  PointStdDevStamped(const PointStdDev & point, const std::int64_t sec, const std::int64_t nsec)
  : point(point)
  {
    stamp = sec * 1e9 + nsec;
  }
  double calc_dist(PointStdDevStamped & other) const
  {
    return (point.pos - other.point.pos).norm();
  }

public:
  PointStdDev point;
  std::int64_t stamp;
};
/**
 * @brief Pose
 */
struct Pose
{
public:
  Pose(float x, float y, float z, float qx, float qy, float qz, float qw)
  {
    Eigen::Matrix3d rotation;
    rotation = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();

    pose = Eigen::Isometry3d::Identity();
    pose.linear() = rotation;
    pose.translation() << x, y, z;
  }

public:
  Eigen::Isometry3d pose;
};
/**
 * @brief Pose stamped
 */
struct PoseStamped
{
public:
  PoseStamped(const Pose & pose, const double sec) : pose(pose)
  {
    stamp = static_cast<std::int64_t>(sec * 1e9);
  }
  PoseStamped(const Pose & pose, const std::int64_t sec, const std::int64_t nsec) : pose(pose)
  {
    stamp = sec * 1e9 + nsec;
  }

public:
  Pose pose;
  std::int64_t stamp;
};
/**
 * @brief configuration struct for FlexCloud packages
 */
struct GeoreferencingConfig
{
  std::string pos_global_path{};
  std::string poses_path{};
  std::string pcd_path{};
  // Point cloud options
  bool include_label{false};
  // Trajectory Alignment
  bool transform_traj{true};
  int rs_num_controlPoints{10};
  double stddev_threshold{0.5};
  std::vector<double> square_size{0.1, 0.1, 10.0};
  // PCD Georeferencing
  std::vector<int64_t> exclude_ind{};
  std::vector<int64_t> shift_ind{};
  std::vector<double> shift_ind_dist{};
  std::vector<int64_t> fake_ind{};
  std::vector<double> fake_ind_dist{};
  std::vector<double> fake_ind_height{};
  int num_cores{4};
  // Zero point
  bool custom_origin{false};
  std::vector<double> origin{0.0, 0.0, 0.0};
};
/**
 * @brief struct to get TUMcolor code from string
 *
 * @param[in] name                  - std::string:
 *                                    name of TUMcolor
 */
struct TUMcolor
{
  explicit TUMcolor(const std::string name)
  {
    // Presentation
    if (name == "Blue") {
      r = 0;
      g = 101;
      b = 189;
      a = 255;
    } else if (name == "Blue1") {
      r = 0;
      g = 51;
      b = 89;
      a = 255;
    } else if (name == "Blue2") {
      r = 0;
      g = 82;
      b = 147;
      a = 255;
    } else if (name == "Blue3") {
      r = 100;
      g = 160;
      b = 200;
      a = 255;
    } else if (name == "Blue4") {
      r = 152;
      g = 198;
      b = 234;
      a = 255;
    } else if (name == "Gray1") {
      r = 51;
      g = 51;
      b = 51;
      a = 255;
    } else if (name == "Gray2") {
      r = 127;
      g = 127;
      b = 127;
      a = 255;
    } else if (name == "Gray3") {
      r = 204;
      g = 204;
      b = 204;
      a = 255;
    } else if (name == "Ivory") {
      r = 218;
      g = 215;
      b = 203;
      a = 255;
    } else if (name == "Orange") {
      r = 227;
      g = 114;
      b = 34;
      a = 255;
    } else if (name == "Green") {
      r = 162;
      g = 173;
      b = 0;
      a = 255;
    } else if (name == "Black") {
      r = 0;
      g = 0;
      b = 0;
      a = 255;
      // Web
    } else if (name == "WEBBlueDark") {
      r = 7;
      g = 33;
      b = 64;
      a = 255;
    } else if (name == "WEBBlueLight") {
      r = 94;
      g = 148;
      b = 212;
      a = 255;
    } else if (name == "WEBYellow") {
      r = 254;
      g = 215;
      b = 2;
      a = 255;
    } else if (name == "WEBOrange") {
      r = 247;
      g = 129;
      b = 30;
      a = 255;
    } else if (name == "WEBPink") {
      r = 181;
      g = 92;
      b = 165;
      a = 255;
    } else if (name == "WEBBlueBright") {
      r = 143;
      g = 129;
      b = 234;
      a = 255;
    } else if (name == "WEBRed") {
      r = 234;
      g = 114;
      b = 55;
      a = 255;
    } else if (name == "WEBGreen") {
      r = 159;
      g = 186;
      b = 54;
      a = 255;
      // otherwise white
    } else {
      r = 255;
      g = 255;
      b = 255;
      a = 255;
    }
  }
  int r, g, b, a;
};
}  // namespace flexcloud
