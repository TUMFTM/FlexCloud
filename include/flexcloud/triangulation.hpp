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
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Delaunay_triangulation_cell_base_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_3.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

typedef CGAL::Triangulation_vertex_base_3<K> Vb3;
typedef CGAL::Delaunay_triangulation_cell_base_3<K> Cb3;
typedef CGAL::Triangulation_data_structure_3<Vb3, Cb3> Tds3;
typedef CGAL::Delaunay_triangulation_3<K, Tds3> DT3;
typedef DT3::Point Point3D;
typedef DT3::Cell_handle CellHandle;
namespace flexcloud
{
/**
 * @brief struct to represent controlpoint
 *
 * @param[in] source                     - Eigen::Vector3d:
 * @param[in] target                     - Eigen::Vector3d:
 */
struct ControlPoint
{
public:
  ControlPoint(const Eigen::Vector3d & source, const Eigen::Vector3d & target)
  : source(source), target(target)
  {
  }

public:
  Eigen::Vector3d source;  // Source point
  Eigen::Vector3d target;  // Target point
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
 * @brief base class for Rubber-sheet transformation
 */
class Delaunay
{
public:
  // Destructor
  virtual ~Delaunay() = default;
  /**
   * @brief calculate enclosing controlpoints from selected ones and trajectory
   *
   * @param[in] size                - std::vector<double>:
   *                                  expansion (percentage)
   * @param[in] target              - std::vector<Eigen::Vector3d>:
   *                                  target trajectory
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  control points on trajectory
   */
  void enclosingControlPoints(
    const std::vector<double> & size, const std::vector<Eigen::Vector3d> & target,
    std::vector<ControlPoint> & cps) const;
  /**
   * @brief insert point into triangulation
   *
   * @param[in] pt                  - Eigen::Vector3d:
   *                                  point to insert
   */
  void insertPoint(const Eigen::Vector3d & pt);
  /**
   * @brief create mapping between control points and triangulation
   *
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  control points
   */
  void mapControlPoints(const std::vector<ControlPoint> & cps);
  /**
   * @brief calculate transformation matrices
   */
  void calcTransformations();
  /**
   * @brief transform single point with rubber-sheeting
   *
   * @param[in] pt                  - Eigen::Vector3d:
   *                                  point to transform
   */
  void transformPoint(Eigen::Vector3d & pt) const;
  /**
   * @brief transform single point with rubber-sheeting in 3D
   *
   * @param[in] pt                  - Eigen::Vector3d:
   *                                  point to transform
   */
  void transformPoint(Eigen::Isometry3d & pose) const;
  /**
   * @brief get edges of triangulation
   *
   * @param[out]                    - std::vector<std::vector<Eigen::Vector3d>>:
   *                                  edges of triangulation
   */
  std::vector<std::vector<Eigen::Vector3d>> getEdges() const;
  /**
   * @brief get vertices of triangulation
   *
   * @param[out]                    - std::vector<std::vector<Eigen::Vector3d>>:
   *                                  vertices of triangulation
   */
  std::vector<std::vector<Eigen::Vector3d>> getVertices() const;

protected:
  /**
   * @brief add pair of source and target points to vector of control points
   *
   * @param[in] target_pt           - std::vector<PointStdDev>:
   *                                  target points
   * @param[in] source_pt           - std::vector<PointStdDev>:
   *                                  source points
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  control points
   */
  void corner2cps(
    const std::vector<Eigen::Vector3d> & target_pt, const std::vector<Eigen::Vector3d> & src_pt,
    std::vector<ControlPoint> & cps) const;
  /**
   * @brief solve linear equations for tetrahedron
   *
   * @param[in] src                 - std::vector<Eigen::Vector3d>:
   *                                  source points
   * @param[in] target              - std::vector<Eigen::Vector3d>:
   *                                  target points
   * @param[out]                    - Eigen::Matrix4d:
   *                                  transformation matrix for tetrahedron
   */
  Eigen::Matrix4d solve_linear_3d(
    const std::vector<Eigen::Vector3d> & src, const std::vector<Eigen::Vector3d> & target);
  // Variables for triangulation, mapping to control points and transformation matrices
  DT3 triangulation_;
  std::vector<CpsMap_3D> mapping_;
  std::vector<Eigen::Matrix4d> mat_;
};
/**
 * @brief add pair of source and target points to vector of control points
 *
 * @param[in] target_pt           - std::vector<Eigen::Vector3d>:
 *                                  target points
 * @param[in] source_pt           - std::vector<Eigen::Vector3d>:
 *                                  source points
 * @param[in] cps                 - std::vector<ControlPoint>:
 *                                  control points
 */
void Delaunay::corner2cps(
  const std::vector<Eigen::Vector3d> & target_pt, const std::vector<Eigen::Vector3d> & src_pt,
  std::vector<ControlPoint> & cps) const
{
  for (size_t i = 0; i < target_pt.size(); ++i) {
    cps.push_back(ControlPoint(src_pt[i], target_pt[i]));
  }
}
/**
 * @brief calculate enclosing cube controlpoints from selected ones and trajectory
 *
 * @param[in] size                - std::vector<double>:
 *                                  expansion (percentage) -> x,y,z
 * @param[in] target              - std::vector<Eigen::Vector3d>:
 *                                  target trajectory
 * @param[in] cps                 - std::vector<ControlPoint>:
 *                                  control points on trajectory
 */
void Delaunay::enclosingControlPoints(
  const std::vector<double> & size, const std::vector<Eigen::Vector3d> & target,
  std::vector<ControlPoint> & cps) const
{
  if (size.size() != 3) {
    std::cout << "Wrong size of boundary definitions!!" << std::endl;
  }
  // Get maximum x, y, and z values from target trajectory
  std::vector<double> x, y, z;

  for (const auto & pt : target) {
    x.push_back(pt.x());
    y.push_back(pt.y());
    z.push_back(pt.z());
  }

  double min_x = *std::min_element(x.begin(), x.end());
  double max_x = *std::max_element(x.begin(), x.end());
  double min_y = *std::min_element(y.begin(), y.end());
  double max_y = *std::max_element(y.begin(), y.end());
  double min_z = *std::min_element(z.begin(), z.end());
  double max_z = *std::max_element(z.begin(), z.end());

  // Set size of cube
  double dx, dy, dz;
  dx = size.at(0) * std::abs(max_x - min_x);
  dy = size.at(1) * std::abs(max_y - min_y);
  dz = size.at(2) * std::abs(max_z - min_z);

  // Define cube
  Eigen::Vector3d v1(min_x - dx, min_y - dy, min_z - dz);
  Eigen::Vector3d v2(min_x - dx, max_y + dy, min_z - dz);
  Eigen::Vector3d v3(max_x + dx, max_y + dy, min_z - dz);
  Eigen::Vector3d v4(max_x + dx, min_y - dy, min_z - dz);
  Eigen::Vector3d v5(min_x - dx, min_y - dy, max_z + dz);
  Eigen::Vector3d v6(min_x - dx, max_y + dy, max_z + dz);
  Eigen::Vector3d v7(max_x + dx, max_y + dy, max_z + dz);
  Eigen::Vector3d v8(max_x + dx, min_y - dy, max_z + dz);

  std::vector<Eigen::Vector3d> pts_target{v1, v2, v3, v4, v5, v6, v7, v8};

  // Calculate points for source
  std::vector<Eigen::Vector3d> pts_src;

  for (size_t i = 0; i < pts_target.size(); i++) {
    std::vector<double> vx, vy, vz, d;

    // Iterate through controlpoints to find closest one
    for (auto & cp : cps) {
      vx.push_back(pts_target.at(i).x() + cp.source.x() - cp.target.x());
      vy.push_back(pts_target.at(i).y() + cp.source.y() - cp.target.y());
      vz.push_back(pts_target.at(i).z() + cp.source.z() - cp.target.z());
      Eigen::Vector3d pt(vx.back(), vy.back(), vz.back());

      d.push_back(std::abs((pt - cp.source).norm()));
    }
    const int ind = std::distance(d.begin(), std::min_element(d.begin(), d.end()));
    Eigen::Vector3d pt_src(vx.at(ind), vy.at(ind), vz.at(ind));
    pts_src.push_back(pt_src);
  }

  // Add points to vector of control points
  this->corner2cps(pts_target, pts_src, cps);
}
/**
 * @brief insert point as 3D point into triangulation
 *
 * @param[in] pt                  - Eigen::Vector3d:
 *                                  point to insert
 */
void Delaunay::insertPoint(const Eigen::Vector3d & pt)
{
  this->triangulation_.insert(Point3D(pt(0), pt(1), pt(2)));
}
/**
 * @brief create mapping between control points and tetrahedrons
 *
 * @param[in] cps                 - std::vector<ControlPoint>:
 *                                  control points
 */
void Delaunay::mapControlPoints(const std::vector<ControlPoint> & cps)
{
  // Iterate over every cell (tetrahedron) and save corresponding control points in struct
  for (DT3::Cell_handle tetra : this->triangulation_.finite_cell_handles()) {
    std::vector<ControlPoint> found_cps;
    // find 4 vertices
    for (size_t i = 0; i < 4; i++) {
      // Check every Controlpoint
      for (auto cp : cps) {
        Point3D p = this->triangulation_.tetrahedron(tetra)[i];
        if (p.x() == cp.target.x() && p.y() == cp.target.y() && p.z() == cp.target.z()) {
          found_cps.push_back(cp);
        }
      }
    }
    if (found_cps.size() != 4)
      std::cout << "\033[1;32m~~~~~~~~~~> Error while mapping CPs to triangulation !\033[0m"
                << std::endl;

    this->mapping_.push_back(CpsMap_3D(found_cps, tetra));
  }
}
/**
 * @brief calculate 4x4 transformation matrices
 */
void Delaunay::calcTransformations()
{
  std::vector<Eigen::Vector3d> target;
  std::vector<Eigen::Vector3d> source;

  for (CpsMap_3D entry : this->mapping_) {
    // Clean up vector
    target.clear();
    source.clear();
    for (size_t i = 0; i < 4; i++) {
      target.push_back(entry.get_cps().at(i).target);
      source.push_back(entry.get_cps().at(i).source);
    }

    // Construct linear equations and solve to find transformation matrix for tetrahedron
    Eigen::Matrix4d mat = solve_linear_3d(source, target);
    this->mat_.push_back(mat);
  }
}
/**
 * @brief transform single point with rubber-sheeting in 3D
 *
 * @param[in] pt                  - Eigen::Vector3d:
 *                                  point to transform
 */
void Delaunay::transformPoint(Eigen::Vector3d & pt) const
{
  // Find area the point is in
  CellHandle cell = this->triangulation_.locate(Point3D(pt(0), pt(1), pt(2)));
  auto map = find_if(this->mapping_.begin(), this->mapping_.end(), [cell](const CpsMap_3D & map) {
    return cell == map.get_cell_handle();
  });
  int index = std::distance(this->mapping_.begin(), map);
  if (this->triangulation_.is_infinite(cell) || index >= static_cast<int>(this->mapping_.size())) {
    // Point outside triangulation
    pt = Eigen::Vector3d(0.0, 0.0, 0.0);
  } else {
    // Point inside triangulation
    const Eigen::Vector4d point(pt(0), pt(1), pt(2), 1.0);
    const Eigen::Vector4d pt_rs = this->mat_.at(index) * point;
    pt = Eigen::Vector3d(pt_rs(0), pt_rs(1), pt_rs(2));
  }
}
/**
 * @brief transform point with 3D transformation
 *
 * @param[in] pt                  - Eigen::Isometry3d
 *                                  point to transform
 */
void Delaunay::transformPoint(Eigen::Isometry3d & pose) const
{
  // Find area the point is in
  CellHandle cell = this->triangulation_.locate(
    Point3D(pose.translation().x(), pose.translation().y(), pose.translation().z()));
  auto map = find_if(this->mapping_.begin(), this->mapping_.end(), [cell](const CpsMap_3D & map) {
    return cell == map.get_cell_handle();
  });
  int index = std::distance(this->mapping_.begin(), map);
  if (this->triangulation_.is_infinite(cell) || index >= static_cast<int>(this->mapping_.size())) {
    // Point outside triangulation
    pose = Eigen::Isometry3d::Identity();
  } else {
    // Point inside triangulation
    pose.matrix() = this->mat_.at(index) * pose.matrix();
  }
}
/**
 * @brief get edges of tetrahedrons
 *
 * @param[out]                    - std::vector<std::vector<Eigen::Vector3d>>:
 *                                  edges of tetrahedrons
 */
std::vector<std::vector<Eigen::Vector3d>> Delaunay::getEdges() const
{
  // Create linestrings and linestring array
  std::vector<std::vector<Eigen::Vector3d>> tet{};

  for (const CpsMap_3D & entry : this->mapping_) {
    CellHandle cell = entry.get_cell_handle();
    // Create points for linestring
    Eigen::Vector3d p1(
      cell->vertex(0)->point().x(), cell->vertex(0)->point().y(), cell->vertex(0)->point().z());
    Eigen::Vector3d p2(
      cell->vertex(1)->point().x(), cell->vertex(1)->point().y(), cell->vertex(1)->point().z());
    Eigen::Vector3d p3(
      cell->vertex(2)->point().x(), cell->vertex(2)->point().y(), cell->vertex(2)->point().z());
    Eigen::Vector3d p4(
      cell->vertex(3)->point().x(), cell->vertex(3)->point().y(), cell->vertex(3)->point().z());

    std::vector<Eigen::Vector3d> ls1{p1, p2};
    std::vector<Eigen::Vector3d> ls2{p1, p3};
    std::vector<Eigen::Vector3d> ls3{p1, p4};
    std::vector<Eigen::Vector3d> ls4{p2, p3};
    std::vector<Eigen::Vector3d> ls5{p2, p4};
    std::vector<Eigen::Vector3d> ls6{p3, p4};
    tet.push_back(ls1);
    tet.push_back(ls2);
    tet.push_back(ls3);
    tet.push_back(ls4);
    tet.push_back(ls5);
    tet.push_back(ls6);
  }
  return tet;
}
/**
 * @brief get vertices of tetrahedrons
 *
 * @param[out]                    - std::vector<std::vector<Eigen::Vector3d>>:
 *                                  vertices of tetrahedrons
 */
std::vector<std::vector<Eigen::Vector3d>> Delaunay::getVertices() const
{
  std::vector<std::vector<Eigen::Vector3d>> vertices;
  std::vector<Eigen::Vector3d> tet;

  for (const CpsMap_3D & entry : this->mapping_) {
    tet.clear();
    for (size_t i = 0; i < 4; ++i) {
      const Eigen::Vector3d p = entry.get_cps().at(i).target;
      tet.push_back(p);
    }
    vertices.push_back(tet);
  }
  return vertices;
}
/**
 * @brief solve linear equations for tetrahedron
 *
 * @param[in] src                 - std::vector<Eigen::Vector3d>:
 *                                  source points
 * @param[in] target              - std::vector<Eigen::Vector3d>:
 *                                  target points
 * @param[out]                    - Eigen::Matrix4d:
 *                                  transformation matrix for tetrahedron
 */
Eigen::Matrix4d Delaunay::solve_linear_3d(
  const std::vector<Eigen::Vector3d> & src, const std::vector<Eigen::Vector3d> & target)
{
  // Construct linear equations
  // b
  Eigen::VectorXd b(16);
  b << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  int i = 0;

  for (auto & pt : src) {
    b(4 * i) = pt(0);
    b(4 * i + 1) = pt(1);
    b(4 * i + 2) = pt(2);
    ++i;
  }
  // A
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(16, 16);
  int j = 0;
  for (auto & pt : target) {
    A.block(4 * j + 0, 0, 1, 4) << pt(0), pt(1), pt(2), 1.0;
    A.block(4 * j + 1, 4, 1, 4) << pt(0), pt(1), pt(2), 1.0;
    A.block(4 * j + 2, 8, 1, 4) << pt(0), pt(1), pt(2), 1.0;
    A.block(4 * j + 3, 12, 1, 4) << pt(0), pt(1), pt(2), 1.0;
    ++j;
  }
  // solve
  const Eigen::VectorXd ti = A.fullPivHouseholderQr().solve(b);

  // Reshape to get matrix
  Eigen::Matrix4d trans;
  trans << ti(0), ti(1), ti(2), ti(3), ti(4), ti(5), ti(6), ti(7), ti(8), ti(9), ti(10), ti(11),
    ti(12), ti(13), ti(14), ti(15);
  return trans;
}
}  // namespace flexcloud
