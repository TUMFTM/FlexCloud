#
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
import os.path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Define path to config folder
    config = os.path.join(
        get_package_share_directory("flexcloud"), "config"
    )

    # Define command line args
    traj_path = DeclareLaunchArgument(
        "traj_path", default_value=TextSubstitution(text="test/Route_1_GPS.txt")
    )
    poses_path = DeclareLaunchArgument(
        "poses_path", default_value=TextSubstitution(text="test/route1_pose_kitti.txt")
    )
    pcd_path = DeclareLaunchArgument(
        "pcd_path", default_value=TextSubstitution(text="pcd_map.pcd")
    )
    pcd_out_path = DeclareLaunchArgument(
        "pcd_out_path", default_value=TextSubstitution(text="pcd_map_georef.pcd")
    )

    # Start nodes
    rviz2_node = Node(
        package="rviz2",
        namespace="",
        executable="rviz2",
        name="rviz2",
        arguments=["-d" + os.path.join(config, "rviz_conf_pcd_georef.rviz")],
    )

    pcd_georef_node = Node(
        package="flexcloud",
        namespace="",
        executable="pcd_georef",
        name="pcd_georef",
        output="screen",
        parameters=[
            {
                "traj_path": LaunchConfiguration("traj_path"),
                "poses_path": LaunchConfiguration("poses_path"),
                "pcd_path": LaunchConfiguration("pcd_path"),
                "pcd_out_path": LaunchConfiguration("pcd_out_path"),
            },
            os.path.join(config, "pcd_georef.param.yaml"),
        ],
    )

    return LaunchDescription(
        [traj_path, poses_path, pcd_path, pcd_out_path, pcd_georef_node, rviz2_node]
    )
