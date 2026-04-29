cmake_minimum_required(VERSION 3.22)
project(flexcloud)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-local-typedefs)
endif()

# Suppress PCL warnings
set(CMAKE_POLICY_DEFAULT_CMP0144 OLD)
set(CMAKE_POLICY_DEFAULT_CMP0167 OLD)

# ament / ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# System dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(GeographicLib REQUIRED)

# Fetch rerun_sdk, yaml-cpp, CLI11
include(FetchContent)
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build rerun_sdk as a shared library" FORCE)
FetchContent_Declare(rerun_sdk URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG 0.8.0
)
FetchContent_Declare(
  CLI11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG v2.4.2
)
FetchContent_MakeAvailable(rerun_sdk yaml-cpp CLI11)

include_directories(SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

####################################
# file_io
####################################
add_library(file_io SHARED
  src/file_io.cpp
)

target_include_directories(file_io
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexcloud>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(file_io
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${Boost_LIBRARIES}
  yaml-cpp::yaml-cpp
  CLI11::CLI11)

####################################
# rosbag_io
####################################
add_library(rosbag_io SHARED
  src/rosbag_io.cpp
)

target_include_directories(rosbag_io
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexcloud>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(rosbag_io
  Eigen3::Eigen
  ${GeographicLib_LIBRARIES})

ament_target_dependencies(rosbag_io
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros)

####################################
# transform_core
####################################
add_library(transform_core SHARED
  src/transform.cpp
)

target_include_directories(transform_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexcloud>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(transform_core
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  CGAL::CGAL
  yaml-cpp::yaml-cpp
  CLI11::CLI11)

####################################
# visualization
####################################
add_library(visualization SHARED
  src/visualization.cpp
)

target_include_directories(visualization
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexcloud>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(visualization
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  CGAL::CGAL
  ${Boost_LIBRARIES}
  rerun_sdk)

####################################
# analysis
####################################
add_library(analysis SHARED
  src/analysis.cpp
)

target_include_directories(analysis
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexcloud>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(analysis
  Eigen3::Eigen)

####################################
# Main Executables
####################################
add_executable(keyframe_interpolation
  src/keyframe_interpolation.cpp
)

target_include_directories(keyframe_interpolation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexcloud>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(keyframe_interpolation
  CLI11::CLI11
  file_io
  rosbag_io
  visualization)

ament_target_dependencies(keyframe_interpolation
  rclcpp)

add_executable(georeferencing
  src/georeferencing.cpp
)

target_include_directories(georeferencing
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexcloud>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(georeferencing
  yaml-cpp::yaml-cpp
  CLI11::CLI11
  file_io
  transform_core
  visualization
  analysis)

####################################
# Install
####################################
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  file_io
  rosbag_io
  transform_core
  visualization
  analysis
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  keyframe_interpolation
  georeferencing
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Python wheel install: also place the executables in bin/ for the wrapper
# launchers, and bake the ROS 2 runtime dependencies into the wheel.
if(SKBUILD)
  install(TARGETS keyframe_interpolation georeferencing RUNTIME DESTINATION bin)
  install(CODE [[
    execute_process(
      COMMAND /bin/bash
              "${CMAKE_SOURCE_DIR}/package/bake_libs.sh"
              "${CMAKE_INSTALL_PREFIX}"
    )
  ]])
endif()

ament_package()
