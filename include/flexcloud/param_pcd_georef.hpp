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

#include <string>
#include <vector>
namespace flexcloud
{
struct FlexCloudConfig
{
  std::string traj_path{};
  std::string poses_path{};
  std::string pcd_path{};
  std::string pcd_out_path{};
  // Transformation GPS - Poses
  int64_t dim{3};
  // Trajectory Alignment
  bool transform_traj{true};
  int rs_num_controlPoints{10};
  double stddev_threshold{0.5};
  std::vector<double> square_size{0.1, 0.1, 10.0};
  // PCD Georeferencing
  bool transform_pcd{true};
  std::vector<int64_t> exclude_ind{};
  std::vector<int64_t> shift_ind{};
  std::vector<double> shift_ind_dist{};
  bool use_threading{true};
  int num_cores{4};
  // Zero point
  bool customZeroPoint{false};
  std::vector<double> zeroPoint{0.0, 0.0, 0.0};
  // Output
  std::string analysis_output_dir{};
};
}  // namespace flexcloud
