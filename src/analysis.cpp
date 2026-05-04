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

#include "analysis.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>
namespace flexcloud
{
std::vector<double> analysis::calc_diff(
  const std::vector<PointStdDevStamped> & src, const std::vector<PoseStamped> & target)
{
  std::vector<double> diff;
  diff.reserve(target.size());
  for (size_t i = 0; i < target.size(); ++i) {
    diff.push_back((target[i].pose.pose.translation() - src[i].point.pos).norm());
  }
  return diff;
}

MatchingStats analysis::compute_stats(const std::vector<double> & diff)
{
  MatchingStats s{};
  if (diff.empty()) return s;

  const double sum = std::accumulate(diff.begin(), diff.end(), 0.0);
  s.mean = sum / static_cast<double>(diff.size());

  double sq_sum = 0.0;
  double var_sum = 0.0;
  for (double d : diff) {
    sq_sum += d * d;
    var_sum += (d - s.mean) * (d - s.mean);
  }
  s.rmse = std::sqrt(sq_sum / static_cast<double>(diff.size()));
  s.stddev = std::sqrt(var_sum / static_cast<double>(diff.size()));

  std::vector<double> sorted = diff;
  std::sort(sorted.begin(), sorted.end());
  const size_t n = sorted.size();
  s.median = (n % 2 == 0) ? 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]) : sorted[n / 2];
  s.min = sorted.front();
  s.max = sorted.back();
  return s;
}

void analysis::print_statistics(const MatchingStats & al, const MatchingStats & rs)
{
  auto row = [](const char * label, double a, double b) {
    std::cout << "  " << std::left << std::setw(8) << label << std::right << std::fixed
              << std::setprecision(4) << std::setw(14) << a << std::setw(14) << b << "\n";
  };

  std::cout << "\033[1;36m=== Trajectory matching statistics (deviation from GNSS, m) ===\033[0m\n";
  std::cout << "\033[32m";
  std::cout << "  " << std::left << std::setw(8) << "" << std::right << std::setw(14) << "Aligned"
            << std::setw(14) << "Rubber-sheet" << "\n";
  row("RMSE",   al.rmse,   rs.rmse);
  row("Mean",   al.mean,   rs.mean);
  row("Median", al.median, rs.median);
  row("Stddev", al.stddev, rs.stddev);
  row("Min",    al.min,    rs.min);
  row("Max",    al.max,    rs.max);
  std::cout << "\033[0m";
}
}  // namespace flexcloud
