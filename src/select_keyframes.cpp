// Copyright 2024 Maximilian Leitenstern

#include "keyframe_interpolation.hpp"
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
  flexcloud::KeyframeInterpolation set_frames(argv[1], argv[2], argv[3], argv[4], argv[5]);
  return 0;
}
