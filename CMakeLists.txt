cmake_minimum_required(VERSION 3.8)
project(flexcloud)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-local-typedefs)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(CGAL REQUIRED)
find_package(GeographicLib REQUIRED)

# Fetch rerun_sdk
include(FetchContent)
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build rerun_sdk as a shared library" FORCE)
FetchContent_Declare(rerun_sdk URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

include_directories(SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

####################################
# file_in
####################################

add_library(file_io SHARED
  src/file_io.cpp
)

target_include_directories(file_io
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexcloud>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(file_io
  rclcpp
  Eigen3
  GeographicLib)
target_link_libraries(file_io
  ${PCL_LIBRARIES}
  ${GeographicLib_LIBRARIES})

####################################
# map_transformation
####################################
add_library(transform SHARED
  src/transform.cpp
)

target_include_directories(transform
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexcloud>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(transform
  rclcpp
  Eigen3)
target_link_libraries(transform
  ${PCL_LIBRARIES}
  CGAL::CGAL)

####################################
# messages
####################################

  add_library(visualization SHARED
  src/visualization.cpp
)

target_include_directories(visualization
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexcloud>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(visualization
  rclcpp
  pcl_conversions)
target_link_libraries(visualization
  ${PCL_LIBRARIES}
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

ament_target_dependencies(analysis
  rclcpp
  Eigen3)

####################################
# Main Executable
####################################

# pcd_georef
add_executable(pcd_georef
  src/pcd_georef.cpp
)

target_include_directories(pcd_georef
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexcloud>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(pcd_georef
  rclcpp)
target_link_libraries(pcd_georef
  file_io
  transform
  visualization
  analysis)

####################################
# Building
####################################

install(
  DIRECTORY lib/
  DESTINATION lib
)

install(TARGETS
  file_io
  transform
  visualization
  analysis
  DESTINATION lib
)

install(TARGETS
  pcd_georef
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
