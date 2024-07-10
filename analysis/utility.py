#!/usr/bin/env python3

# TUM Autonomous Motorsport Georeferencing Tool
# Copyright (C) 2024 Maximilian Leitenstern, Marko Alten, Christian Bolea-Schaser
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


def load_file(dir_path, file_name, deli):
    if os.path.exists(os.path.join(dir_path, file_name)):
        var = np.transpose(
            np.genfromtxt(os.path.join(dir_path, file_name), delimiter=deli)
        )
        return var
    else:
        print("Cannot find " + file_name)


def load_file_man(dir_path, file_name, deli):
    if os.path.exists(os.path.join(dir_path, file_name)):
        out = []
        lengths = []
        with open(os.path.join(dir_path, file_name), "r") as file:
            for line in file:
                line_split = line.split(deli)
                if line_split[-1] == "\n":
                    line_split.pop()
                out.append(line_split)
                lengths.append(len(line_split))
        lim = np.max(lengths)
        for li in out:
            while len(li) < lim:
                li.append("nan")
        return np.array(out, dtype=float)
    else:
        print("Cannot find " + file_name)


def load_file_str(dir_path, file_name):
    if os.path.exists(os.path.join(dir_path, file_name)):
        out = []
        with open(os.path.join(dir_path, file_name), "r") as file:
            for line in file:
                out.append(line.strip())
        return out
    else:
        print("Cannot find " + file_name)


def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

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
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


# Define TUM color codes
def TUMcolor(name, alpha=1.0):
    if name == "Blue":
        col = [0.0 / 255.0, 101.0 / 255.0, 189.0 / 255.0, alpha]
    elif name == "Blue1":
        col = [0.0 / 255.0, 51.0 / 255.0, 89.0 / 255.0, alpha]
    elif name == "Blue2":
        col = [0.0 / 255.0, 82.0 / 255.0, 147.0 / 255.0, alpha]
    elif name == "Blue3":
        col = [100.0 / 255.0, 160.0 / 255.0, 200.0 / 255.0, alpha]
    elif name == "Blue4":
        col = [152.0 / 255.0, 198.0 / 255.0, 234.0 / 255.0, alpha]
    elif name == "Gray1":
        col = [51.0 / 255.0, 51.0 / 255.0, 51.0 / 255.0, alpha]
    elif name == "Gray2":
        col = [127.0 / 255.0, 127.0 / 255.0, 127.0 / 255.0, alpha]
    elif name == "Gray3":
        col = [204.0 / 255.0, 204.0 / 255.0, 204.0 / 255.0, alpha]
    elif name == "Ivory":
        col = [218.0 / 255.0, 215.0 / 255.0, 203.0 / 255.0, alpha]
    elif name == "Orange":
        col = [227.0 / 255.0, 114.0 / 255.0, 34.0 / 255.0, alpha]
    elif name == "Green":
        col = [162.0 / 255.0, 173.0 / 255.0, 0.0 / 255.0, alpha]
    elif name == "Black":
        col = [0.0 / 255.0, 0.0 / 255.0, 0.0 / 255.0, alpha]
    elif name == "WEBBlueDark":
        col = [7.0 / 255.0, 33.0 / 255.0, 64.0 / 255.0, alpha]
    elif name == "WEBBlueLight":
        col = [94.0 / 255.0, 148.0 / 255.0, 212.0 / 255.0, alpha]
    elif name == "WEBYellow":
        col = [254.0 / 255.0, 215.0 / 255.0, 2.0 / 255.0, alpha]
    elif name == "WEBOrange":
        col = [247.0 / 255.0, 129.0 / 255.0, 30.0 / 255.0, alpha]
    elif name == "WEBPink":
        col = [181.0 / 255.0, 92.0 / 255.0, 165.0 / 255.0, alpha]
    elif name == "WEBRed":
        col = [217.0 / 255.0, 81.0 / 255.0, 23.0 / 255.0, alpha]
    elif name == "WEBGreen":
        col = [159.0 / 255.0, 186.0 / 255.0, 54.0 / 255.0, alpha]
    else:
        col = [255.0 / 255.0, 255.0 / 255.0, 255.0 / 255.0, alpha]
    return col
