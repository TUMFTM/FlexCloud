<div align="center">

<h1>FlexCloud</h1>

Georeferencing of Point Cloud Maps

[![Linux](https://img.shields.io/badge/os-linux-blue.svg)](https://www.linux.org/)
[![Docker](https://badgen.net/badge/icon/docker?icon=docker&label)](https://www.docker.com/)
[![ROS2humble](https://img.shields.io/badge/ros2-humble-blue.svg)](https://docs.ros.org/en/humble/index.html)

</div>

<h2>Overview</h2>
This project enables the georeferencing of an existing point cloud map created only from inertial sensor data (e.g. LiDAR) by the use of the corresponding GNSS data.
Leveraging the concept of rubber-sheeting from 2D cartography, the tool is also able to account for accumulated errors during map creation and thus rectyfy the map.


<h2>Usage</h2>

* check parameters for georeferencing in `/config/pcd_georef.param.yaml`

1. Necessary input parameters:
   - `traj_path` => path to GNSS trajectory of the vehicle (format: txt-file with lat, lon, ele, lat_stddev, lon_stddev, ele_stddev)
   - `poses_path` => path to SLAM trajectory of the vehicle (KITTI-format, resulting from the previous step)
   - `pcd_path` => path to exported pcd map corresponding to poses trajectory (resulting from the previous step)
   - `pcd_out_path` => path to save the final, georeferenced point cloud map (DEFAULT: /pcd_map_georef.pcd)

2. Start the package
   ```bash
       ros2 launch tam_pcd_georeferencing pcd_georef.launch.py traj_path:=<path-to-GPS-trajectory> poses_path:=<path-to-SLAM-trajectory>  pcd_path:=<path-to-pcd-map> pcd_out_path:=<path-to-save-pcd-map>
   ```

3. Optional: Select control points
   - after the trajectories are loaded and the target trajectory is roughly aligned to the master trajectory you are asked in the command window to select control points for the rubber-sheet transformation (the amount of points can be configured in the config file).
   - select the desired points using the `Publish Point` button in RVIZ and follow the instructions in the console.

4. Inspect results
   - results of the rubber-sheet transformation & the resulting, transformed point cloud map are visualized.
   - adjust the parameters if the results are satisfying
   - see table for explanation of single topics
   - run the script described in [Analysis](doc/analysis.md)
   - Quick usage (the directory output/traj_matching is automatically generated at the current working directory):
   ```bash
       plot_traj_matching.py /path/to/output/traj_matching/
   ```

| Topic | Description |
| ----------- | ----------- |
| `/tam/traj/traj_markers` | GNSS Trajectory |
| `/tam/traj/traj_SLAM_markers` | original SLAM rajectory |
| `/tam/traj/traj_align_markers` | SLAM trajectory aligned to GPS with [Umeyama](https://web.stanford.edu/class/cs273/refs/umeyama.pdf) transformation |
| `/tam/traj/traj_rs_markers` | SLAM trajectory after [rubber-sheet](https://www.tandfonline.com/doi/abs/10.1559/152304085783915135)-transformation |
| `/tam/rs/geom_markers_cps` | control points used for rubber-sheeting |
| `/tam/rs/geom_markers_triag` | triangulation used for rubber-sheeting |
| `/clicked_point` | last 2 selected points by user to indicate chosen control point |
| `/tam/rs/pcd_map` | transformed point cloud map |

   - Inspect results and modify parameters if desired.

<h2>Content</h2>

Detailed documentation of the modules can be found below.

1. [Geometric Alignment](doc/alignment.md)

2. [Analysis](doc/analysis.md)
