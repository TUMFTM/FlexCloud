
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

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "utility.hpp"
namespace flexcloud
{
struct MatchingStats
{
  double rmse;
  double mean;
  double median;
  double stddev;
  double min;
  double max;
};

class analysis
{
public:
  analysis() {}

  /**
   * @brief calculate per-point euclidean deviation of a target trajectory
   *        relative to a (paired-by-index) source trajectory
   */
  std::vector<double> calc_diff(
    const std::vector<PointStdDevStamped> & src, const std::vector<PoseStamped> & target);

  /**
   * @brief reduce a deviation vector to summary statistics
   */
  MatchingStats compute_stats(const std::vector<double> & diff);

  /**
   * @brief pretty-print aligned and rubber-sheeted matching statistics to stdout
   */
  void print_statistics(const MatchingStats & al, const MatchingStats & rs);
};
}  // namespace flexcloud
