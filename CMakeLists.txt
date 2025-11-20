cmake_minimum_required(VERSION 3.8)
project(flexcloud)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-local-typedefs)
endif()

# find dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(CGAL REQUIRED)
find_package(Boost REQUIRED)
find_package(GeographicLib REQUIRED)

# Fetch rerun_sdk and yaml-cpp
include(FetchContent)
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build rerun_sdk as a shared library" FORCE)
FetchContent_Declare(rerun_sdk URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG 0.8.0
)
FetchContent_MakeAvailable(rerun_sdk yaml-cpp)

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

target_link_libraries(file_io
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${Boost_LIBRARIES})

####################################
# map_transformation
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
  CGAL::CGAL)

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
  transform_core
  visualization
  analysis
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

####################################
# Main Executables
####################################

# select_keyframes
add_executable(keyframe_interpolation
  src/keyframe_interpolation.cpp
)

target_include_directories(keyframe_interpolation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexcloud>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(keyframe_interpolation
  yaml-cpp::yaml-cpp
  file_io
  visualization)

# pcd_georef
add_executable(georeferencing
  src/georeferencing.cpp
)

target_include_directories(georeferencing
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/flexcloud>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(georeferencing
  yaml-cpp::yaml-cpp
  file_io
  transform_core
  visualization
  analysis)

install(TARGETS
  georeferencing
  keyframe_interpolation
  DESTINATION flexcloud)
