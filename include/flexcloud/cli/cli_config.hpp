/*
 * TUM Autonomous Motorsport Georeferencing Tool
 * Copyright (C) 2026 Maximilian Leitenstern
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "CLI/CLI.hpp"
#include "yaml-cpp/yaml.h"

namespace flexcloud::config
{
/**
 * @brief CLI-driven configuration for the keyframe_interpolation executable.
 *
 * All algorithm parameters are CLI options.
 */
struct KeyframeInterpolationConfig
{
  // Required
  std::string positions_path{};
  std::string poses_path{};
  std::string out_dir{"."};

  // Bag-only options (used when positions_path resolves to a ROS 2 bag)
  std::string pos_topic{};
  std::string target_frame{};
  std::vector<double> origin{};  // [lat, lon, alt] for NavSatFix

  // Algorithm parameters
  float stddev_threshold{5.0f};
  float keyframe_delta_x{2.0f};
  float keyframe_delta_angle{0.5f};
  bool interpolate{true};
  float interp_pos_delta_xyz{0.25f};

  void add_cli_options(CLI::App * app)
  {
    app->add_option(
         "positions-path", positions_path,
         "Reference-data source. Auto-detected: a single .txt file is read line-by-line "
         "(\"stamp x y z x_stddev y_stddev z_stddev\"); a directory of per-position .txt "
         "files (filename = \"<sec>_<nanosec>.txt\") is read as such; a .mcap / .db3 / "
         ".sqlite3 file or a ROS 2 bag directory is read as a bag.")
      ->required()
      ->group("Required");

    app->add_option("poses-path", poses_path, "Path to SLAM trajectory in GLIM format")
      ->required()
      ->check(CLI::ExistingFile)
      ->group("Required");

    app->add_option(
         "out-dir", out_dir,
         "Output directory for poses_keyframes.txt and positions_interpolated.txt")
      ->capture_default_str()
      ->group("Required");

    app->add_option("--pos-topic", pos_topic,
                    "Topic name (NavSatFix or Odometry) — required when positions-path is a bag")
      ->group("Bag input");

    app->add_option("-t,--target-frame", target_frame,
                    "Target TF frame to transform positions into "
                    "(uses /tf and /tf_static from bag)")
      ->group("Bag input");

    app->add_option("--origin", origin,
                    "Custom origin for local Cartesian projection [lat lon alt]")
      ->expected(3)
      ->type_name("FLOAT FLOAT FLOAT")
      ->group("Bag input");

    app->add_option("--stddev-threshold", stddev_threshold,
                    "Reject reference frames whose horizontal stddev exceeds this value")
      ->capture_default_str()
      ->group("Algorithm");

    app->add_option("--keyframe-delta-x", keyframe_delta_x,
                    "Minimum translational distance between keyframes [m]")
      ->capture_default_str()
      ->group("Algorithm");

    app->add_option("--keyframe-delta-angle", keyframe_delta_angle,
                    "Minimum rotational distance between keyframes [rad]")
      ->capture_default_str()
      ->group("Algorithm");

    app->add_flag("--interpolate,!--no-interpolate", interpolate,
                  "Spline-interpolate the reference position at each keyframe timestamp "
                  "(otherwise use closest-neighbor selection)")
      ->capture_default_str()
      ->group("Algorithm");

    app->add_option("--interp-pos-delta-xyz", interp_pos_delta_xyz,
                    "Minimum euclidean distance between consecutive spline support points [m]")
      ->capture_default_str()
      ->group("Algorithm");
  }
};

/**
 * @brief Configuration struct for the georeferencing executable.
 */
struct GeoreferencingConfig
{
  // Inputs / outputs
  std::string positions_path{};
  std::string poses_path{};
  std::string pcd_path{};
  std::string config_file{};  // optional YAML for index arrays

  // Trajectory matching
  int control_points{10};
  double stddev_threshold{0.05};
  std::vector<double> square_size{0.1, 0.1, 10.0};
  std::vector<double> origin{};

  // Index-based fine tuning (YAML-only)
  std::vector<int64_t> exclude_ind{};
  std::vector<int64_t> shift_ind{};
  std::vector<double> shift_ind_dist{};
  std::vector<int64_t> fake_ind{};
  std::vector<double> fake_ind_dist{};
  std::vector<double> fake_ind_height{};

  void add_cli_options(CLI::App * app)
  {
    app->add_option("positions-path", positions_path, "Path to GNSS / reference trajectory")
      ->required()
      ->check(CLI::ExistingFile)
      ->group("Required");

    app->add_option("poses-path", poses_path, "Path to SLAM trajectory in GLIM format")
      ->required()
      ->check(CLI::ExistingFile)
      ->group("Required");

    app->add_option("--pcd", pcd_path,
                    "Optional point cloud map to transform alongside the trajectory")
      ->group("Inputs");

    app->add_option("--config-file", config_file,
                    "Optional YAML file with index-based fine-tuning arrays "
                    "(exclude_ind, shift_ind, shift_ind_dist, fake_ind, fake_ind_dist, "
                    "fake_ind_height)")
      ->check(CLI::ExistingFile)
      ->group("Inputs");

    app->add_option("--control-points", control_points,
                    "Number of control points for rubber-sheeting")
      ->capture_default_str()
      ->group("Trajectory matching");

    app->add_option(
         "--stddev-threshold", stddev_threshold,
         "Maximum stddev of reference points for automatic control-point selection")
      ->capture_default_str()
      ->group("Trajectory matching");

    app->add_option("--square-size", square_size,
                    "Padding of enclosing square around trajectories [x y z] (fractions)")
      ->expected(3)
      ->type_name("FLOAT FLOAT FLOAT")
      ->capture_default_str()
      ->group("Trajectory matching");

    app->add_option("--origin", origin, "Custom ENU zero point [lat lon alt]")
      ->expected(3)
      ->type_name("FLOAT FLOAT FLOAT")
      ->capture_default_str()
      ->group("Origin");
  }

  /**
   * @brief Overlay index-based arrays from the YAML referenced by
   *        @ref config_file. Missing keys leave the corresponding field unchanged.
   */
  void load_yaml_overlay()
  {
    if (config_file.empty()) {
      return;
    }
    YAML::Node y = YAML::LoadFile(config_file);
    auto load_int = [&](const char * key, std::vector<int64_t> & dst) {
      if (y[key]) dst = y[key].as<std::vector<int64_t>>();
    };
    auto load_dbl = [&](const char * key, std::vector<double> & dst) {
      if (y[key]) dst = y[key].as<std::vector<double>>();
    };
    load_int("exclude_ind", exclude_ind);
    load_int("shift_ind", shift_ind);
    load_dbl("shift_ind_dist", shift_ind_dist);
    load_int("fake_ind", fake_ind);
    load_dbl("fake_ind_dist", fake_ind_dist);
    load_dbl("fake_ind_height", fake_ind_height);
  }
};
}  // namespace flexcloud::config
