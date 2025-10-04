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
#include <iostream>

#include "keyframe_interpolation.hpp"
#include "utility.hpp"
int main(int argc, char * argv[])
{
  // Check the number of arguments
  if (argc < 5) {
    // Tell the user how to run the program
    std::cerr << "Usage: " << argv[0]
              << " <config_path> <pos_dir_path> <kitti_odom_path> <pcd_dir_path> <dst_dir_path>"
              << std::endl;
    return 1;
  }
  if (argc == 5) {
    // Only interpolation without accumulating clouds
    flexcloud::KeyframeInterpolation set_frames(argv[1], argv[2], argv[3], argv[4]);
    set_frames.visualize();
  }
  if (argc == 6) {
    // Interpolation with accumulating clouds
    flexcloud::KeyframeInterpolation set_frames(argv[1], argv[2], argv[3], argv[4], argv[5]);
    set_frames.visualize();
  }
  return 0;
}
