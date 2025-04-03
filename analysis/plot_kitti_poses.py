#!/usr/bin/env python3

# TUM Autonomous Motorsport Georeferencing Tool
# Copyright (C) 2024 Maximilian Leitenstern, Dominik Kulmer
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import os
import numpy as np
import argparse
import matplotlib.pyplot as plt


def set_axes_equal(ax):
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([(z_middle - plot_radius), (z_middle + plot_radius)])


def main(file_path, gt_path, equal_axis_flag):
    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    poses = np.loadtxt(file_path)

    # Reshape get translation values
    tx, ty, tz = [], [], []

    for pose in poses:
        tx.append(pose[3])
        ty.append(pose[7])
        tz.append(pose[11])

    # Read GT data
    if (gt_path is not None):
        gt_tx, gt_ty, gt_tz = [], [], []

        if os.path.isdir(gt_path):
            # Read GT data from multiple files in a directory
            for gt_file in sorted(os.listdir(gt_path)):
                gt_file_path = os.path.join(gt_path, gt_file)
                with open(gt_file_path, 'r') as f:
                    line = f.readline().strip()
                    x, y, z, stddev_x, stddev_y, stddev_z = map(float, line.split())
                    gt_tx.append(x)
                    gt_ty.append(y)
                    gt_tz.append(z)
        else:
            # Read GT data from a single file
            gt_data = np.loadtxt(gt_path)
            gt_tx = gt_data[:, 0]
            gt_ty = gt_data[:, 1]
            gt_tz = gt_data[:, 2]

        # Plot Ground Truth
        ax.scatter3D(gt_tx, gt_ty, gt_tz, color="red", label="Ground Truth")

    # Plot LiDAR trajectory
    ax.scatter3D(tx, ty, tz, color="blue", label="LiDAR Trajectory")
    ax.plot3D(tx, ty, tz, color="blue")

    # Set axis labels
    ax.set_xlabel("X in m")
    ax.set_ylabel("Y in m")
    ax.set_zlabel("Z in m")
    ax.set_title("LiDAR Trajectory and Ground Truth")

    # Set equal scaling
    ax.set_box_aspect([1, 1, 1])  # Aspect ratio is 1:1:1

    ax.view_init(azim=-90, elev=90)

    # Show legend
    ax.legend()

    # [optional] Set axes equal
    if equal_axis_flag:
        set_axes_equal(ax)

    # Set projection type
    ax.set_proj_type('ortho')  # FOV = 0 deg

    # Show the plot
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot a LiDAR trajectory in KITTI format and the ground truth in TUM MapLoc format.')  # noqa: E501
    parser.add_argument('trajectory', type=str, help='Input directory with .txt files containing the LiDAR trajectory in KITTI format')  # noqa: E501
    parser.add_argument('ground_truth', nargs='?', type=str, help='[Optional] Input directory or single .txt file for the ground truth in TUM MapLoc format')  # noqa: E501
    parser.add_argument('--equal_axis', action='store_true', help="Flag to scale all axes equally.")  # noqa: E501

    if not parser.parse_args().ground_truth:
        parser.parse_args().ground_truth = None

    main(parser.parse_args().trajectory,
         parser.parse_args().ground_truth,
         parser.parse_args().equal_axis)
