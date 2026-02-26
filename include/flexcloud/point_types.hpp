/*
 * TUM Autonomous Motorsport Georeferencing Tool
 * Copyright (C) 2026 Andreas Weber
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
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

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIL,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t, label, label))
