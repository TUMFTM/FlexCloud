/*
 * Custom PCL point types for FlexCloud
 */
#pragma once

#include <pcl/point_types.h>

#include <cstdint>
#include <pcl/impl/point_types.hpp>
// A point type that contains XYZ + intensity + label
struct EIGEN_ALIGN16 PointXYZIL
{
  PCL_ADD_POINT4D;  // quad-word XYZ
  float intensity;
  std::uint32_t label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Register the PointXYZIL type so PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIL,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t, label, label))
