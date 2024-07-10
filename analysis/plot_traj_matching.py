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

import sys
from matplotlib.collections import LineCollection
import matplotlib.pyplot as plt
import numpy as np
from utility import TUMcolor, load_file, set_axes_equal


class TrajAnalysis:
    def __init__(self, dir_path, dim):
        # Read files
        self.source = load_file(dir_path, "source.txt", " ")
        self.target = load_file(dir_path, "target.txt", " ")
        self.target_al = load_file(dir_path, "target_al.txt", " ")
        self.target_rs = load_file(dir_path, "target_rs.txt", " ")
        self.triag = load_file(dir_path, "triag.txt", " ")
        self.controlPoints = load_file(dir_path, "controlPoints.txt", " ")
        self.diff_al = load_file(dir_path, "diff_al.txt", "")
        self.diff_rs = load_file(dir_path, "diff_rs.txt", "")
        # Set dimension of transform
        self.dim = dim

    # Plot initial trajectories in local coordinate system
    def plot_traj(self):
        fig = plt.figure(figsize=plt.figaspect(0.3))
        ax = fig.add_subplot(1, 3, 1)
        ax.plot(
            self.source[0][0:-1:1],
            self.source[1][0:-1:1],
            label="GNSS trajectory",
            color=TUMcolor("Green"),
        )
        ax.plot(
            self.target[0][0:-1:1],
            self.target[1][0:-1:1],
            label="SLAM trajectory",
            color=TUMcolor("Blue"),
        )
        ax.set_xlabel("$X$ in m", fontsize=11)
        ax.set_ylabel("$Y$ in m", fontsize=11)
        ax.legend()
        ax.grid()
        ax.set_aspect("equal")

        # Plot initial trajectories z coordinate
        ax = fig.add_subplot(1, 3, 2)
        ind = np.arange(self.source[2].size)
        ax.plot(
            ind[0:-1:1],
            self.source[2][0:-1:1],
            label="GNSS trajectory",
            color=TUMcolor("Green"),
        )
        ax.plot(
            ind[0:-1:1],
            self.target[2][0:-1:1],
            label="SLAM trajectory",
            color=TUMcolor("Blue"),
        )
        ax.set_xlabel("Point Index", fontsize=11)
        ax.set_ylabel("$Z$ in m", fontsize=11)
        ax.legend()
        ax.grid()
        ax.set_aspect("equal")

        # Plot initial trajectories in local coordinate system 3D
        ax = fig.add_subplot(1, 3, 3, projection="3d")
        ax.plot3D(
            self.source[0][0:-1:1],
            self.source[1][0:-1:1],
            self.source[2][0:-1:1],
            label="GNSS trajectory",
            color=TUMcolor("Green"),
        )
        ax.plot3D(
            self.target[0][0:-1:1],
            self.target[1][0:-1:1],
            self.target[2][0:-1:1],
            label="SLAM trajectory",
            color=TUMcolor("Blue"),
        )
        ax.set_xlabel("$X$ in m", fontsize=11)
        ax.set_ylabel("$Y$ in m", fontsize=11)
        ax.set_zlabel("$Z$ in m", fontsize=11)
        ax.grid()
        ax.set_box_aspect([1.0, 1.0, 1.0])
        set_axes_equal(ax)

    # Plot trajectories after rigid transformation with deviation
    def plot_traj_al(self):
        # Setup plot
        diff_al_norm = plt.Normalize(self.diff_al.min(), self.diff_al.max())
        diff_al_points = self.target_al[:2].T.reshape(-1, 1, 2)
        diff_al_segments = np.concatenate(
            [diff_al_points[:-1], diff_al_points[1:]], axis=1
        )
        diff_al_lines = LineCollection(diff_al_segments, cmap="jet", norm=diff_al_norm)
        diff_al_lines.set_array(self.diff_al)

        fig = plt.figure(figsize=plt.figaspect(0.3))

        # Plot xy-coordinates after rigid alignment
        ax = fig.add_subplot(1, 3, 1)
        ax.plot(
            self.source[0][0:-1:1],
            self.source[1][0:-1:1],
            label="GNSS trajectory",
            color=TUMcolor("Gray2"),
        )
        ax.add_collection(diff_al_lines)
        ax.set_xlabel("$X$ in m", fontsize=11)
        ax.set_ylabel("$Y$ in m", fontsize=11)
        ax.legend()
        ax.grid()
        ax.set_aspect("equal")

        # Plot z-coordinate after rigid alignment
        ax = fig.add_subplot(1, 3, 2)
        ind = np.arange(self.source[2].size)
        ax.plot(
            ind[0:-1:1],
            self.source[2][0:-1:1],
            label="GNSS trajectory",
            color=TUMcolor("Green"),
        )
        ax.plot(
            ind[0:-1:1],
            self.target_al[2][0:-1:1],
            label="Umeyama aligned trajectory",
            color=TUMcolor("Blue"),
        )
        ax.set_xlabel("Point Index", fontsize=11)
        ax.set_ylabel("$Z$ in m", fontsize=11)
        ax.legend()
        ax.grid()
        ax.set_aspect("auto")

        # Colorbar
        cax = fig.add_axes(
            [
                ax.get_position().x0 - 0.1,
                ax.get_position().y0,
                0.02,
                ax.get_position().height,
            ]
        )
        cbar = fig.colorbar(diff_al_lines, cax=cax)
        cbar.set_label("Deviation from GPS in m", fontsize=11)

        ax = fig.add_subplot(1, 3, 3, projection="3d")
        ax.plot3D(
            self.source[0][0:-1:1],
            self.source[1][0:-1:1],
            self.source[2][0:-1:1],
            label="GNSS trajectory",
            color=TUMcolor("Green"),
        )
        ax.plot3D(
            self.target_al[0][0:-1:1],
            self.target_al[1][0:-1:1],
            self.target_al[2][0:-1:1],
            label="Umeyama aligned trajectory",
            color=TUMcolor("Blue"),
        )
        ax.set_xlabel("$X$ in m", fontsize=11)
        ax.set_ylabel("$Y$ in m", fontsize=11)
        ax.set_zlabel("$Z$ in m", fontsize=11)
        ax.legend()
        ax.grid()
        ax.set_box_aspect([1.0, 1.0, 1.0])
        set_axes_equal(ax)

    # Plot trajectories after rubber-sheet transformation with deviation
    def plot_traj_rs(self):
        # Setup plot
        diff_rs_norm = plt.Normalize(self.diff_rs.min(), self.diff_rs.max())
        diff_rs_points = self.target_rs[:2].T.reshape(-1, 1, 2)
        diff_rs_segments = np.concatenate(
            [diff_rs_points[:-1], diff_rs_points[1:]], axis=1
        )
        diff_rs_lines = LineCollection(diff_rs_segments, cmap="jet", norm=diff_rs_norm)
        diff_rs_lines.set_array(self.diff_rs)

        # Plot xy
        fig = plt.figure(figsize=plt.figaspect(0.3))
        ax = fig.add_subplot(1, 3, 1)
        ax.plot(
            self.source[0][0:-1:1],
            self.source[1][0:-1:1],
            label="GPS trajectory",
            color=TUMcolor("Gray2"),
        )
        ax.add_collection(diff_rs_lines)
        ax.set_xlabel("$X$ in m", fontsize=11)
        ax.set_ylabel("$Y$ in m", fontsize=11)
        ax.legend()
        ax.grid()
        ax.set_aspect("equal")
        cax = fig.add_axes(
            [
                ax.get_position().x0 - 0.1,
                ax.get_position().y0,
                0.02,
                ax.get_position().height,
            ]
        )
        cbar = fig.colorbar(diff_rs_lines, cax=cax)
        cbar.set_label("Deviation from GPS in m", fontsize=11)

        # Plot z-coordinate after rigid alignment
        ax = fig.add_subplot(1, 3, 2)
        ind = np.arange(self.source[2].size)
        ax.plot(
            ind[0:-1:1],
            self.source[2][0:-1:1],
            label="GNSS trajectory",
            color=TUMcolor("Green"),
        )
        ax.plot(
            ind[0:-1:1],
            self.target_rs[2][0:-1:1],
            label="Rubber-sheeted trajectory",
            color=TUMcolor("Blue"),
        )
        ax.set_xlabel("Point Index", fontsize=11)
        ax.set_ylabel("$Z$ in m", fontsize=11)
        ax.legend()
        ax.grid()
        ax.set_aspect("auto")

        # Plot 3d
        ax = fig.add_subplot(1, 3, 3, projection="3d")
        ax.plot3D(
            self.source[0][0:-1:1],
            self.source[1][0:-1:1],
            self.source[2][0:-1:1],
            label="GNSS trajectory",
            color=TUMcolor("Green"),
        )
        ax.plot3D(
            self.target_rs[0][0:-1:1],
            self.target_rs[1][0:-1:1],
            self.target_rs[2][0:-1:1],
            label="Rubber-sheeted trajectory",
            color=TUMcolor("Orange"),
        )
        ax.set_xlabel("$X$ in m", fontsize=11)
        ax.set_ylabel("$Y$ in m", fontsize=11)
        ax.set_ylabel("$Z$ in m", fontsize=11)
        ax.legend()
        ax.grid()
        ax.set_box_aspect([1.0, 1.0, 1.0])
        set_axes_equal(ax)

    # Plot geometry of rubber-sheet transformation (triangles, control points)
    def plot_geom_rs(self, draw_triag=False):
        fig = plt.figure(figsize=plt.figaspect(0.3))

        # Plot xy
        ax = fig.add_subplot(1, 3, 1)
        ax.plot(
            self.source[0][0:-1:1],
            self.source[1][0:-1:1],
            label="GNSS trajectory",
            color=TUMcolor("Green"),
        )
        ax.plot(
            self.target_al[0][0:-1:1],
            self.target_al[1][0:-1:1],
            label="Umeyama aligned trajectory",
            color=TUMcolor("Blue"),
        )
        if draw_triag:
            for i in range(np.size(self.triag, 1)):
                for j in range(np.size(self.triag, 0) // 3 - 1):
                    ax.plot(
                        [self.triag[3 * j + 0][i], self.triag[3 * j + 3][i]],
                        [self.triag[3 * j + 1][i], self.triag[3 * j + 4][i]],
                        linewidth=1,
                        color=TUMcolor("Green"),
                    )
                # Plot connection from first to last vertex
                ax.plot(
                    [self.triag[0][i], self.triag[-3][i]],
                    [self.triag[1][i], self.triag[-2][i]],
                    linewidth=1,
                    color=TUMcolor("Green"),
                )
        for i in range(np.size(self.controlPoints, 1)):
            ax.plot(
                [self.controlPoints[0][i], self.controlPoints[3][i]],
                [self.controlPoints[1][i], self.controlPoints[4][i]],
                marker="*",
                color=TUMcolor("Orange"),
            )

        ax.set_xlabel("$X$ in m", fontsize=11)
        ax.set_ylabel("$Y$ in m", fontsize=11)
        ax.legend()
        ax.grid()
        ax.set_aspect("equal")

        # Plot xy
        ax = fig.add_subplot(1, 3, 2)
        ax.plot(
            self.source[0][0:-1:1],
            self.source[2][0:-1:1],
            label="GNSS trajectory",
            color=TUMcolor("Green"),
        )
        ax.plot(
            self.target_al[0][0:-1:1],
            self.target_al[2][0:-1:1],
            label="Umeyama aligned trajectory",
            color=TUMcolor("Blue"),
        )
        if draw_triag:
            for i in range(np.size(self.triag, 1)):
                for j in range(np.size(self.triag, 0) // 3 - 1):
                    ax.plot(
                        [self.triag[3 * j + 0][i], self.triag[3 * j + 3][i]],
                        [self.triag[3 * j + 2][i], self.triag[3 * j + 5][i]],
                        linewidth=1,
                        color=TUMcolor("Green"),
                    )
                # Plot connection from first to last vertex
                ax.plot(
                    [self.triag[0][i], self.triag[-3][i]],
                    [self.triag[2][i], self.triag[-1][i]],
                    linewidth=1,
                    color=TUMcolor("Green"),
                )
        for i in range(np.size(self.controlPoints, 1)):
            ax.plot(
                [self.controlPoints[0][i], self.controlPoints[3][i]],
                [self.controlPoints[2][i], self.controlPoints[5][i]],
                marker="*",
                color=TUMcolor("Orange"),
            )

        ax.set_xlabel("$X$ in m", fontsize=11)
        ax.set_ylabel("$Z$ in m", fontsize=11)
        ax.legend()
        ax.grid()
        ax.set_aspect("equal")

        # Plot xy
        ax = fig.add_subplot(1, 3, 3, projection="3d")
        ax.plot3D(
            self.source[0][0:-1:1],
            self.source[1][0:-1:1],
            self.source[2][0:-1:1],
            label="GNSS trajectory",
            color=TUMcolor("Green"),
        )
        ax.plot3D(
            self.target_al[0][0:-1:1],
            self.target_al[1][0:-1:1],
            self.target_al[2][0:-1:1],
            label="Umeyama aligned trajectory",
            color=TUMcolor("Blue"),
        )
        if draw_triag:
            for i in range(np.size(self.triag, 1)):
                for j in range(np.size(self.triag, 0) // 3 - 1):
                    ax.plot3D(
                        [self.triag[3 * j + 0][i], self.triag[3 * j + 3][i]],
                        [self.triag[3 * j + 1][i], self.triag[3 * j + 4][i]],
                        [self.triag[3 * j + 2][i], self.triag[3 * j + 5][i]],
                        linewidth=1,
                        color=TUMcolor("Green"),
                    )
                # Plot connection from first to last vertex
                ax.plot3D(
                    [self.triag[0][i], self.triag[-3][i]],
                    [self.triag[1][i], self.triag[-2][i]],
                    [self.triag[2][i], self.triag[-1][i]],
                    linewidth=1,
                    color=TUMcolor("Green"),
                )
        for i in range(np.size(self.controlPoints, 1)):
            ax.plot3D(
                [self.controlPoints[0][i], self.controlPoints[3][i]],
                [self.controlPoints[1][i], self.controlPoints[4][i]],
                [self.controlPoints[2][i], self.controlPoints[5][i]],
                marker="*",
                color=TUMcolor("Orange"),
            )

        ax.set_xlabel("$X$ in m", fontsize=11)
        ax.set_ylabel("$Y$ in m", fontsize=11)
        ax.set_ylabel("$Z$ in m", fontsize=11)
        ax.legend()
        ax.grid()
        ax.set_box_aspect([1.0, 1.0, 1.0])
        set_axes_equal(ax)

    # Statistical values on deviations
    def print_statistics(self):
        rmse_al = np.sqrt(np.mean(self.diff_al**2))
        rmse_rs = np.sqrt(np.mean(self.diff_rs**2))
        print("\033[1;36mMatching statistics:\033[0m")

        print("\033[32mRMSE aligned target trajectory: " + str(rmse_al) + "\033[0m")
        print("\033[32mRMSE rubber-sheeted target trajectory: " + str(rmse_rs))
        print(
            "\033[32mMean aligned target trajectory: "
            + str(np.mean(self.diff_al))
            + "\033[0m"
        )
        print(
            "\033[32mMean rubber-sheeted target trajectory: "
            + str(np.mean(self.diff_rs))
            + "\033[0m"
        )
        print(
            "\033[32mMedian aligned target trajectory: "
            + str(np.median(self.diff_al))
            + "\033[0m"
        )
        print(
            "\033[32mMedian rubber-sheeted target trajectory: "
            + str(np.median(self.diff_rs))
            + "\033[0m"
        )
        print(
            "\033[32mStandard deviation aligned target trajectory: "
            + str(np.std(self.diff_al))
            + "\033[0m"
        )
        print(
            "\033[32mStandard deviation rubber-sheeted target trajectory: "
            + str(np.std(self.diff_rs))
            + "\033[0m"
        )


# Main function - comment and uncomment functions you want to use
if __name__ == "__main__":
    # Check command-line arguments
    if len(sys.argv) != 3:
        print("Usage: plot_traj_matching.py  <dim> <path/to/output/traj_matching>")
        sys.exit(1)

    # Get command-line arguments
    dim = sys.argv[1]
    dir_path = sys.argv[2]
    # Init
    traj = TrajAnalysis(dir_path, dim)

    traj.plot_traj()
    traj.plot_traj_al()
    traj.plot_traj_rs()

    # optional: set draw_triag to True to show triangulation in rubber-sheet geometry
    traj.plot_geom_rs(draw_triag=True)
    traj.print_statistics()

    plt.show()
