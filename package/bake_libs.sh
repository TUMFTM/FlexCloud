#!/bin/bash
# Copies the ROS 2 shared libraries and ament index entries that the
# FlexCloud executables need at runtime into the wheel install prefix, so
# `pip install flexcloud` works on a stock Ubuntu without a system ROS
# installation. Invoked from CMakeLists.txt during a SKBUILD install.
#
# Adapted from PointCloudCrafter (Apache 2.0).
# https://github.com/TUMFTM/PointCloudCrafter

shopt -s nullglob

PREFIX=$1
DEST_LIB="$PREFIX/lib"
DEST_SHARE="$PREFIX/share"
ROS_PATH="/opt/ros/jazzy"

mkdir -p "$DEST_LIB" "$DEST_SHARE"

# Copy each pattern in turn; nullglob means a non-matching pattern produces
# zero files instead of being passed literally to cp.
copy_glob() {
  local files=( $1 )
  if [ ${#files[@]} -gt 0 ]; then
    cp -P "${files[@]}" "$DEST_LIB/"
  fi
}

# RMW + storage plugins that auditwheel can't reach via symbol references.
copy_glob "$ROS_PATH/lib/librmw_fastrtps_*.so*"
copy_glob "$ROS_PATH/lib/librosbag2_storage_*.so*"
copy_glob "$ROS_PATH/lib/libfast*.so*"
copy_glob "$ROS_PATH/lib/libmcap*.so*"

# Full ament index
if [ -d "$ROS_PATH/share/ament_index" ]; then
  cp -rp "$ROS_PATH/share/ament_index/." "$DEST_SHARE/ament_index/"
fi

# Packages we need at runtime: those declared in our package.xml plus a fixed
# baseline of ROS 2 core/plugin packages that pluginlib walks at load time.
PARENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/.. &> /dev/null && pwd )"
PKGS=()
if [ -f "$PARENT_DIR/package.xml" ]; then
  PKGS=($(grep -E '<(depend|exec_depend)>' "$PARENT_DIR/package.xml" | \
      sed -E 's/.*<(depend|exec_depend)>([^<]+)<\/(depend|exec_depend)>.*/\2/' | sort -u))
fi
EXTRA_PKGS=(
    "rmw" "rmw_implementation" "rcl" "rclcpp" "fastrtps" "fastcdr"
    "std_msgs" "builtin_interfaces" "geometry_msgs" "action_msgs" "unique_identifier_msgs"
    "rcl_interfaces" "rosgraph_msgs" "statistics_msgs" "tf2_msgs" "tf2"
    "rosbag2_cpp"
    "rosbag2_storage" "rosbag2_storage_default_plugins" "rosbag2_storage_mcap"
    "rosbag2_storage_sqlite3"
    "rosbag2_compression" "rosbag2_compression_zstd"
    "ament_index" "rcutils" "pluginlib" "class_loader"
    "rosidl_typesupport_cpp" "rosidl_typesupport_c"
    "rosidl_typesupport_fastrtps_c" "rosidl_typesupport_fastrtps_cpp"
    "rosidl_typesupport_introspection_c" "rosidl_typesupport_introspection_cpp"
    "sensor_msgs" "nav_msgs"
)
ALL_PKGS=("${PKGS[@]}" "${EXTRA_PKGS[@]}")

for pkg in "${ALL_PKGS[@]}"; do
  if [ -d "$ROS_PATH/share/$pkg" ]; then
    cp -aL "$ROS_PATH/share/$pkg" "$DEST_SHARE/" 2>/dev/null \
      || echo "[bake_libs] warning: failed to copy share/$pkg"
  fi
  find "$ROS_PATH/lib" -maxdepth 1 \
      -name "lib${pkg}*.so*" \
      ! -name "*__rosidl_generator_py.so" \
      -exec cp -P {} "$DEST_LIB/" \; 2>/dev/null || true
done

# Trim test plugins / build-time files that aren't needed at runtime
find "$DEST_SHARE/ament_index/resource_index" \
    -name "rosbag2_storage" \
    -path "*plugin*" \
    -delete 2>/dev/null || true
find "$DEST_LIB" "$DEST_SHARE" -name "*.cmake" -delete 2>/dev/null || true
find "$DEST_SHARE" -name "*.pc" -delete 2>/dev/null || true
find "$DEST_SHARE" -name "*ConfigVersion.cmake" -delete 2>/dev/null || true
find "$DEST_SHARE" -name "*Targets.cmake" -delete 2>/dev/null || true
