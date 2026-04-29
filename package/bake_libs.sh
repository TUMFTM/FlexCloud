#!/bin/bash
# Copies the ROS 2 shared libraries and ament index entries that the
# FlexCloud executables need at runtime into the wheel install prefix, so
# `pip install flexcloud` works on a stock Ubuntu without a system ROS
# installation. Invoked from CMakeLists.txt during a SKBUILD install.
#
# Adapted from PointCloudCrafter (Apache 2.0).

set -e

PREFIX=$1
DEST_LIB="$PREFIX/lib"
DEST_SHARE="$PREFIX/share"
ROS_PATH="/opt/ros/jazzy"

# Plugins not picked up by auditwheel (no symbol references)
cp "$ROS_PATH/lib/librmw_fastrtps_"*.so* "$DEST_LIB/"
cp "$ROS_PATH/lib/librosbag2_storage_default_plugins.so"* "$DEST_LIB/"
cp "$ROS_PATH/lib/librosbag2_storage_mcap.so"* "$DEST_LIB/"
cp "$ROS_PATH/lib/libfast"*.so* "$DEST_LIB/"

# Full ament index
if [ -d "$ROS_PATH/share/ament_index" ]; then
    cp -rp "$ROS_PATH/share/ament_index/." "$DEST_SHARE/ament_index/"
fi

PARENT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/.. &> /dev/null && pwd )"
PKGS=($(grep -E '<(depend|exec_depend)>' "$PARENT_DIR/package.xml" | \
    sed -E 's/.*<(depend|exec_depend)>([^<]+)<\/(depend|exec_depend)>.*/\2/' | sort -u))
EXTRA_PKGS=(
    "rmw" "rmw_implementation" "rcl" "rclcpp" "fastrtps" "fastcdr"
    "std_msgs" "builtin_interfaces" "geometry_msgs" "action_msgs" "unique_identifier_msgs"
    "rcl_interfaces" "rosgraph_msgs" "statistics_msgs" "tf2_msgs" "tf2"
    "rosbag2_storage" "rosbag2_storage_default_plugins" "rosbag2_storage_mcap"
    "rosbag2_storage_sqlite3"
    "rosbag2_compression" "rosbag2_compression_zstd"
    "ament_index" "rcutils" "pluginlib"
    "rosidl_typesupport_cpp" "rosidl_typesupport_c"
    "rosidl_typesupport_fastrtps_c" "rosidl_typesupport_fastrtps_cpp"
    "rosidl_typesupport_introspection_c" "rosidl_typesupport_introspection_cpp"
    "sensor_msgs" "nav_msgs"
)
ALL_PKGS=("${PKGS[@]}" "${EXTRA_PKGS[@]}")

for pkg in "${ALL_PKGS[@]}"; do
    if [ -d "$ROS_PATH/share/$pkg" ]; then
        cp -aL "$ROS_PATH/share/$pkg" "$DEST_SHARE/"
    fi

    find "$ROS_PATH/lib" \
        -name "lib${pkg}*.so*" \
        ! -name "*__rosidl_generator_py.so" \
        -exec cp -P {} "$DEST_LIB/" \;
done

# Trim test plugins / build-time files that aren't needed at runtime
find "$DEST_SHARE/ament_index/resource_index" \
    -name "rosbag2_storage" \
    -path "*plugin*" \
    -delete 2>/dev/null || true
find "$DEST_LIB" "$DEST_SHARE" -name "*.cmake" -delete
find "$DEST_SHARE" -name "*.pc" -delete
find "$DEST_SHARE" -name "*ConfigVersion.cmake" -delete
find "$DEST_SHARE" -name "*Targets.cmake" -delete
