cmake_minimum_required(VERSION 3.8)
project(FlexCloud)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-local-typedefs)
endif()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")

# intermediate triangulation viewer
add_definitions(-DCGAL_USE_BASIC_VIEWER)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(CGAL REQUIRED)
find_package(GeographicLib REQUIRED)

include_directories(SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

file(MAKE_DIRECTORY lib)

####################################
# file_in
####################################

add_library(file_io SHARED
  src/file_io.cpp
)

target_include_directories(file_io
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/FlexCloud>
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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/FlexCloud>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(transform
  geometry_msgs
  rclcpp
  Eigen3)
target_link_libraries(transform
  ${PCL_LIBRARIES}
  CGAL::CGAL)

####################################
# messages
####################################

  add_library(messages SHARED
  src/messages.cpp
)

target_include_directories(messages
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/FlexCloud>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(messages
  rclcpp
  geometry_msgs
  visualization_msgs
  pcl_conversions)
target_link_libraries(messages
  ${PCL_LIBRARIES})

####################################
# analysis
####################################

add_library(analysis SHARED
  src/analysis.cpp
)

target_include_directories(analysis
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/FlexCloud>
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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/FlexCloud>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(pcd_georef
  rclcpp
  geometry_msgs
  visualization_msgs)
target_link_libraries(pcd_georef
  file_io
  transform
  messages
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
  messages
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
