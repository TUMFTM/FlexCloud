/*
 * TUM Autonomous Motorsport Georeferencing Tool
 * Copyright (C) 2024 Maximilian Leitenstern, Marko Alten, Christian Bolea-Schaser
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

#pragma once
//
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Delaunay_triangulation_cell_base_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_2.h>
#include <CGAL/Triangulation_vertex_base_3.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <string>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

typedef CGAL::Triangulation_vertex_base_3<K> Vb3;
typedef CGAL::Delaunay_triangulation_cell_base_3<K> Cb3;
typedef CGAL::Triangulation_data_structure_3<Vb3, Cb3> Tds3;
typedef CGAL::Delaunay_triangulation_3<K, Tds3> DT3;
typedef DT3::Point Point3D;
typedef DT3::Cell_handle CellHandle;

typedef CGAL::Triangulation_vertex_base_2<K> Vb2;
typedef CGAL::Triangulation_face_base_2<K> Fb2;
typedef CGAL::Triangulation_data_structure_2<Vb2, Fb2> Tds2;
typedef CGAL::Delaunay_triangulation_2<K, Tds2> DT2;
typedef DT2::Point Point2D;
typedef DT2::Face_handle FaceHandle;
/**
 * @brief struct to represent gps point with stddevs
 *
 * @param[in] lat                   - double:
 *                                    lateral degree
 * @param[in] lon                   - double:
 *                                    longitudinal degree
 * @param[in] ele                   - double:
 *                                    elevation
 * @param[in] lat_stddev            - double:
 *                                    lateral standard deviation
 * @param[in] lon_stddev            - double:
 *                                    longitudinal standard deviation
 * @param[in] ele_stddev            - double:
 *                                    elevation standard deviation
 */
struct GPSPoint
{
public:
  GPSPoint(
    const double lat, const double lon, const double ele, const double lat_stddev,
    const double lon_stddev, const double ele_stddev)
  : lat_(lat),
    lon_(lon),
    ele_(ele),
    lat_stddev_(lat_stddev),
    lon_stddev_(lon_stddev),
    ele_stddev_(ele_stddev)
  {
  }
  double lat_{0.};
  double lon_{0.};
  double ele_{0.};
  double lat_stddev_{0.};
  double lon_stddev_{0.};
  double ele_stddev_{0.};
};
/**
 * @brief struct to represent metric position with standard deviation
 *
 * @param[in] x                     - double:
 *                                    x-position
 * @param[in] y                     - double:
 *                                    y-position
 * @param[in] z                     - double:
 *                                    z-position
 * @param[in] x_stddev              - double:
 *                                    x standard deviation
 * @param[in] y_stddev              - double:
 *                                    y standard deviation
 * @param[in] z_stddev              - double:
 *                                    z standard deviation
 */
struct ProjPoint
{
public:
  ProjPoint(
    const double x, const double y, const double z, const double x_stddev, const double y_stddev,
    const double z_stddev)
  : pos_(x, y, z), stddev_(x_stddev, y_stddev, z_stddev)
  {
  }
  Eigen::Vector3d pos_;
  Eigen::Vector3d stddev_;
};
/**
 * @brief struct to represent controlpoint
 *
 * @param[in] vx                    - double:
 *                                    x-pos of source point
 * @param[in] vy                    - double:
 *                                    y-pos of source point
 * @param[in] vz                    - double:
 *                                    z-pos of source point
 * @param[in] ux                    - double:
 *                                    x-pos of source point
 * @param[in] uy                    - double:
 *                                    y-pos of source point
 * @param[in] uz                    - double:
 *                                    z-pos of source point
 */
struct ControlPoint
{
public:
  ControlPoint(
    const double vx, const double vy, const double vz, const double ux, const double uy,
    const double uz)
  : v_(vx, vy, vz), u_(ux, uy, uz)
  {
  }
  Eigen::Vector3d get_source_point() const { return this->v_; }
  Eigen::Vector3d get_target_point() const { return this->u_; }

private:
  Eigen::Vector3d v_;  // Source point
  Eigen::Vector3d u_;  // Target point
};
/**
 * @brief struct to represent mapping of controlpoints to triangulation
 *
 * @param[in] cps                   - std::vector<ControlPoint>:
 *                                    vector of control points of corresponding triangle
 * @param[in] fh                    - FaceHandle:
 *                                    face handle of triangle
 */
struct CpsMap_2D
{
public:
  CpsMap_2D(std::vector<ControlPoint> cps, FaceHandle & fh) : cps_(cps), face_handle_(fh) {}
  std::vector<ControlPoint> get_cps() const { return this->cps_; }
  FaceHandle get_face_handle() const { return this->face_handle_; }

private:
  std::vector<ControlPoint> cps_;
  FaceHandle face_handle_;
};
/**
 * @brief struct to represent mapping of controlpoints to triangulation
 *
 * @param[in] cps                   - std::vector<ControlPoint>:
 *                                    vector of control points of corresponding tetrahedron
 * @param[in] ch                    - CellHandle:
 *                                    cell handle of tetrahedron
 */
struct CpsMap_3D
{
public:
  CpsMap_3D(std::vector<ControlPoint> cps, CellHandle & ch) : cps_(cps), cell_handle_(ch) {}
  std::vector<ControlPoint> get_cps() const { return this->cps_; }
  CellHandle get_cell_handle() const { return this->cell_handle_; }

private:
  std::vector<ControlPoint> cps_;
  CellHandle cell_handle_;
};
/**
 * @brief struct to get TUMcolor code from string
 *
 * @param[in] name                  - std::string:
 *                                    name of TUMcolor
 */
struct TUMcolor
{
  explicit TUMcolor(const std::string name)
  {
    // Presentation
    if (name == "Blue") {
      r = 0;
      g = 101;
      b = 189;
      a = 255;
    } else if (name == "Blue1") {
      r = 0;
      g = 51;
      b = 89;
      a = 255;
    } else if (name == "Blue2") {
      r = 0;
      g = 82;
      b = 147;
      a = 255;
    } else if (name == "Blue3") {
      r = 100;
      g = 160;
      b = 200;
      a = 255;
    } else if (name == "Blue4") {
      r = 152;
      g = 198;
      b = 234;
      a = 255;
    } else if (name == "Gray1") {
      r = 51;
      g = 51;
      b = 51;
      a = 255;
    } else if (name == "Gray2") {
      r = 127;
      g = 127;
      b = 127;
      a = 255;
    } else if (name == "Gray3") {
      r = 204;
      g = 204;
      b = 204;
      a = 255;
    } else if (name == "Ivory") {
      r = 218;
      g = 215;
      b = 203;
      a = 255;
    } else if (name == "Orange") {
      r = 227;
      g = 114;
      b = 34;
      a = 255;
    } else if (name == "Green") {
      r = 162;
      g = 173;
      b = 0;
      a = 255;
    } else if (name == "Black") {
      r = 0;
      g = 0;
      b = 0;
      a = 255;
      // Web
    } else if (name == "WEBBlueDark") {
      r = 7;
      g = 33;
      b = 64;
      a = 255;
    } else if (name == "WEBBlueLight") {
      r = 94;
      g = 148;
      b = 212;
      a = 255;
    } else if (name == "WEBYellow") {
      r = 254;
      g = 215;
      b = 2;
      a = 255;
    } else if (name == "WEBOrange") {
      r = 247;
      g = 129;
      b = 30;
      a = 255;
    } else if (name == "WEBPink") {
      r = 181;
      g = 92;
      b = 165;
      a = 255;
    } else if (name == "WEBBlueBright") {
      r = 143;
      g = 129;
      b = 234;
      a = 255;
    } else if (name == "WEBRed") {
      r = 234;
      g = 114;
      b = 55;
      a = 255;
    } else if (name == "WEBGreen") {
      r = 159;
      g = 186;
      b = 54;
      a = 255;
      // otherwise white
    } else {
      r = 255;
      g = 255;
      b = 255;
      a = 255;
    }
  }
  int r, g, b, a;
};
;
