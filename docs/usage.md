# Usage

FlexCloud exposes two executables; all algorithm parameters are CLI flags with sensible defaults.

## Application

After installation (see [Installation](installation.md)), a typical end-to-end run looks like:

```bash
# 1. interpolate reference positions at SLAM keyframes
flexcloud-keyframe-interpolation reference.txt poses_GLIM.txt out/

# 2. georeference the trajectory (and optionally a point cloud map)
flexcloud-georeferencing out/positions_interpolated.txt out/poses_keyframes.txt \
    --pcd map.pcd --evaluation
```

The `--evaluation` flag additionally prints quantitative matching statistics to the
terminal and visualises per-segment GNSS deviation as a jet-colormap-shaded linestring in the Rerun viewer.

## `flexcloud-keyframe-interpolation`

Select keyframes from a SLAM trajectory and interpolate matching reference positions,
producing inputs for the georeferencing step.

```text
flexcloud-keyframe-interpolation [OPTIONS] <positions-path> <poses-path> [out-dir]

Required positional arguments:
  positions-path  Reference data (auto-detected, see table below)
  poses-path      SLAM trajectory in GLIM format
                  (one row per pose: stamp xpos ypos zpos xquat yquat zquat wquat)
  out-dir         Output directory for poses_keyframes.txt, positions_interpolated.txt and
                  keyframe_interpolation.yaml (defaults to the current directory)

Bag input (only used when positions-path is a ROS 2 bag):
  --pos-topic TEXT           Topic of NavSatFix or Odometry messages (required for bags)
  -t, --target-frame TEXT    Frame of the SLAM poses (e.g. base_link). Reference positions are
                             moved into this frame using /tf and /tf_static from the bag.
                             Optional.

Origin:
  --origin LAT LON ALT       Origin of the local ENU frame, ALT above mean sea level.
                             NavSatFix: defaults to the first accepted fix. txt input:
                             positions are read as lat lon ellipsoidal-alt if given, as
                             Cartesian coordinates otherwise.

Algorithm (excerpt):
  --max-time-diff FLOAT [0.5] Maximum time [s] between a keyframe and the nearest reference
                             position (on both sides when interpolating); other keyframes
                             are skipped. Needs reference data of at least 1/max-time-diff Hz.
```

Besides the two trajectory files, `keyframe_interpolation.yaml` records the effective
parameters, including the origin of the ENU frame (also when it was taken from the first fix),
which is needed to place the georeferenced map in global coordinates.

### Reference-data auto-detection

The reader is selected automatically from `positions-path`:

| `positions-path` is … | Reader |
| --- | --- |
| a `.txt` file | text-file reader (one position per line) |
| a directory **without** any `.mcap` / `.db3` / `.sqlite3` | per-position txt-files reader |
| a `.mcap` / `.db3` / `.sqlite3` file, **or** a directory containing one | ROS 2 bag reader |

### Reference-data file formats

- **single `.txt` file** — one position per line, whitespace-separated:
  `stamp x y z x_stddev y_stddev z_stddev`.
- **directory of per-position `.txt` files** — filenames `<sec>_<nanosec>.txt`,
  contents `x y z x_stddev y_stddev z_stddev` (timestamp parsed from the filename).
- **ROS 2 bag** — supports `sensor_msgs/msg/NavSatFix` or `nav_msgs/msg/Odometry`.

!!! note "Notes on bag input"
    - `NavSatFix` messages are projected to local Cartesian via
      [GeographicLib](https://geographiclib.sourceforge.io/2009-03/classGeographicLib_1_1LocalCartesian.html);
      standard deviations are taken from `position_covariance` (diagonal). Fixes without
      a GNSS solution (`status < STATUS_FIX` or non-finite coordinates) are skipped; this
      includes the default `STATUS_UNKNOWN`, so the GNSS driver has to set the status.
    - The NavSatFix altitude is ellipsoidal; the geoid height (EGM2008) at the origin is
      subtracted, so z is the height above mean sea level relative to the origin. Without
      `--origin`, the first accepted fix becomes the origin and is logged.
    - `Odometry` messages use `pose.pose` directly; stddevs come from
      `pose.covariance` (diagonal).
    - When `--target-frame` is set, all `/tf` and `/tf_static` messages are buffered
      first. Once the whole bag has been read, each position is shifted by the lever arm
      from its frame to the target frame, rotated with the vehicle orientation from TF at
      the message timestamp. The bag's TF tree must hold the full chain from the world
      frame to the target frame; messages without a transform are dropped (and counted).
        - `Odometry`: world frame `header.frame_id`, lever arm from `child_frame_id`.
        - `NavSatFix`: lever arm from `header.frame_id` (the antenna). The fix names no
          world frame, so the root of the antenna's TF tree is used; it must be
          ENU-aligned like the projection (e.g. a local cartesian frame), its origin
          does not matter. It also has to be connected to the antenna through at least
          one moving `/tf` transform; with only static transforms the root belongs to the
          vehicle and the run stops with an error.

### Examples

```bash
# single txt file
flexcloud-keyframe-interpolation positions.txt poses_GLIM.txt

# ROS 2 bag with Odometry
flexcloud-keyframe-interpolation /path/to/bag.mcap poses_GLIM.txt /path/to/out \
    --pos-topic /odom --target-frame base_link
```

### How keyframes are selected

- Keyframes are selected from the LiDAR trajectory based on minimum longitudinal
  distance (`keyframe_delta_x`) or minimum angular delta (`keyframe_delta_angle`).
- For each LiDAR keyframe, the corresponding reference position is computed in one of
  two ways (controlled by `interpolate`):
    - **Closest neighbour** — pick the reference frame with the smallest timestamp delta.
    - **Spline interpolation** — fit a third-order spline through neighbouring reference
      points (selected so consecutive supports have a minimum euclidean distance of
      `interp_pos_delta_xyz`) and evaluate at the keyframe timestamp.
- Keyframes without reference data close in time are skipped (and counted): the nearest
  reference position has to be within `max_time_diff`, on both sides of the keyframe when
  interpolating. This drops keyframes before or after the reference recording and inside
  gaps of it. The reference data therefore needs at least 1/`max_time_diff` Hz (2 Hz with
  the default 0.5 s); 5 Hz or more is recommended.
- `stddev_threshold` drops reference frames with high covariance.

The output is designed to be consumed directly by `flexcloud-georeferencing`.

---

## `flexcloud-georeferencing`

Georeference a SLAM trajectory and (optionally) a corresponding point cloud map by
aligning to a GNSS / reference trajectory using Umeyama plus rubber-sheeting.

```text
Usage: georeferencing [OPTIONS] positions-path poses-path

Required:
  positions-path TEXT:FILE REQUIRED   Path to GNSS / reference trajectory
  poses-path     TEXT:FILE REQUIRED   Path to SLAM trajectory in GLIM format

Options:
  -h, --help                          Print this help message and exit

Inputs:
  --pcd TEXT                  Optional point cloud map to transform alongside the
                              trajectory
  --config-file TEXT:FILE     Optional YAML file with index-based fine-tuning arrays
                              (exclude_ind, shift_ind, shift_ind_dist, fake_ind,
                              fake_ind_dist, fake_ind_height)

Trajectory matching:
  --control-points INT:POSITIVE [10]
                              Number of control points for rubber-sheeting
  --stddev-threshold FLOAT [0.25]
                              Maximum stddev of reference points for automatic
                              control-point selection
  --square-size FLOAT FLOAT FLOAT [0.1,0.1,10] ...
                              Padding of enclosing square around trajectories
                              [x y z] (fractions)

Origin:
  --origin FLOAT FLOAT FLOAT  Custom ENU zero point [lat lon alt], alt above mean
                              sea level; the positions are then read as lat lon
                              ellipsoidal-alt

Evaluation:
  --evaluation                Print trajectory-matching statistics (RMSE, mean,
                              median, stddev of GNSS-vs-aligned and
                              GNSS-vs-rubber-sheeted deviations) to the terminal
                              and log per-segment, deviation-colored linestrings
                              to the Rerun viewer.
```

### Examples

```bash
# cartesian reference, no point cloud, default parameters
flexcloud-georeferencing positions_interpolated.txt poses_keyframes.txt

# GPS reference, custom origin, transform a point cloud as well
flexcloud-georeferencing reference.txt poses_keyframes.txt \
    --pcd map.pcd --origin 48.262 11.667 0.0

# supply index-based fine-tuning arrays via YAML
flexcloud-georeferencing reference.txt poses_keyframes.txt \
    --config-file georeferencing.yaml
```

### Index-based fine-tuning YAML

The only YAML configuration that remains is for the index-based fine-tuning arrays
(`exclude_ind`, `shift_ind`, `shift_ind_dist`, `fake_ind`, `fake_ind_dist`,
`fake_ind_height`), supplied via `--config-file`.

When `--pcd` is provided and the map can be loaded, the effective configuration is dumped
next to the input point cloud as `georeferencing.yaml` (alongside the transformed `georef_<pcd>`
output). The dumped file can be edited and fed back in via `--config-file` for
reproducible runs.
