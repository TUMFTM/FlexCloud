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
#include <Eigen/Geometry>
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "utility.hpp"
namespace flexcloud
{
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
   * @param[in] target              - std::vector<ProjPoint>:
   *                                  target trajectory
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  control points on trajectory
   */
  virtual void enclosingControlPoints(
    const std::vector<double> & size, const std::vector<ProjPoint> & target,
    std::vector<ControlPoint> & cps) const = 0;
  /**
   * @brief insert point into triangulation
   *
   * @param[in] pt                  - Eigen::Vector3d:
   *                                  point to insert
   */
  virtual void insertPoint(const Eigen::Vector3d & pt) = 0;
  /**
   * @brief create mapping between control points and triangulation
   *
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  control points
   */
  virtual void mapControlPoints(const std::vector<ControlPoint> & cps) = 0;
  /**
   * @brief calculate transformation matrices
   */
  virtual void calcTransformations() = 0;
  /**
   * @brief transform single point with rubber-sheeting
   *
   * @param[in] pt                  - Eigen::Vector3d:
   *                                  point to transform
   */
  virtual void transformPoint(Eigen::Vector3d & pt) const = 0;
  /**
   * @brief get edges of triangulation
   *
   * @param[out]                    - std::vector<std::vector<ProjPoint>>:
   *                                  edges of triangulation
   */
  virtual std::vector<std::vector<ProjPoint>> getEdges() const = 0;
  /**
   * @brief get vertices of triangulation
   *
   * @param[out]                    - std::vector<std::vector<ProjPoint>>:
   *                                  vertices of triangulation
   */
  virtual std::vector<std::vector<ProjPoint>> getVertices() const = 0;

protected:
  /**
   * @brief add pair of source and target points to vector of control points
   *
   * @param[in] target_pt           - std::vector<ProjPoint>:
   *                                  target points
   * @param[in] source_pt           - std::vector<ProjPoint>:
   *                                  source points
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  control points
   */
  void corner2cps(
    const std::vector<Eigen::Vector3d> & target_pt, const std::vector<Eigen::Vector3d> & src_pt,
    std::vector<ControlPoint> & cps) const;
};
/**
 * @brief interhited class for 2D Rubber-sheet transformation
 */
class Delaunay_2D : public Delaunay
{
public:
  // Constructor
  Delaunay_2D() {}
  /**
   * @brief calculate enclosing rectangle controlpoints from selected ones and trajectory
   *
   * @param[in] size                - std::vector<double>:
   *                                  expansion (percentage) -> x,y
   * @param[in] target              - std::vector<ProjPoint>:
   *                                  target trajectory
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  control points on trajectory
   */
  void enclosingControlPoints(
    const std::vector<double> & size, const std::vector<ProjPoint> & target,
    std::vector<ControlPoint> & cps) const override;
  /**
   * @brief insert point as 2D point into triangulation
   *
   * @param[in] pt                  - Eigen::Vector3d:
   *                                  point to insert
   */
  void insertPoint(const Eigen::Vector3d & pt) override;
  /**
   * @brief create mapping between control points and triangles
   *
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  control points
   */
  void mapControlPoints(const std::vector<ControlPoint> & cps) override;
  /**
   * @brief calculate 3x3 transformation matrices
   */
  void calcTransformations() override;
  /**
   * @brief transform single point with rubber-sheeting in 2D
   *
   * @param[in] pt                  - Eigen::Vector3d:
   *                                  point to transform
   */
  void transformPoint(Eigen::Vector3d & pt) const override;
  /**
   * @brief get edges of triangles
   *
   * @param[out]                    - std::vector<std::vector<ProjPoint>>:
   *                                  edges of triangles
   */
  std::vector<std::vector<ProjPoint>> getEdges() const override;
  /**
   * @brief get vertices of triangles
   *
   * @param[out]                    - std::vector<std::vector<ProjPoint>>:
   *                                  vertices of triangles
   */
  std::vector<std::vector<ProjPoint>> getVertices() const override;

private:
  /**
   * @brief solve linear equations for triangle
   *
   * @param[in] src                 - std::vector<Eigen::Vector3d>:
   *                                  source points
   * @param[in] target              - std::vector<Eigen::Vector3d>:
   *                                  target points
   * @param[out]                    - Eigen::Matrix3d:
   *                                  transformation matrix for triangle
   */
  Eigen::Matrix3d solve_linear_2d(
    const std::vector<Eigen::Vector3d> & src, const std::vector<Eigen::Vector3d> & target);
  // Variables for triangulation, mapping to control points and transformation matrices
  DT2 triangulation_;
  std::vector<CpsMap_2D> mapping_;
  std::vector<Eigen::Matrix3d> mat_;
};
/**
 * @brief interhited class for 3D Rubber-sheet transformation
 */
class Delaunay_3D : public Delaunay
{
public:
  // Constructor
  Delaunay_3D() {}
  /**
   * @brief calculate enclosing cube controlpoints from selected ones and trajectory
   *
   * @param[in] size                - std::vector<double>:
   *                                  expansion (percentage) -> x,y,z
   * @param[in] target              - std::vector<ProjPoint>:
   *                                  target trajectory
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  control points on trajectory
   */
  void enclosingControlPoints(
    const std::vector<double> & size, const std::vector<ProjPoint> & target,
    std::vector<ControlPoint> & cps) const override;
  /**
   * @brief insert point as 3D point into triangulation
   *
   * @param[in] pt                  - Eigen::Vector3d:
   *                                  point to insert
   */
  void insertPoint(const Eigen::Vector3d & pt) override;
  /**
   * @brief create mapping between control points and tetrahedrons
   *
   * @param[in] cps                 - std::vector<ControlPoint>:
   *                                  control points
   */
  void mapControlPoints(const std::vector<ControlPoint> & cps) override;
  /**
   * @brief calculate 4x4 transformation matrices
   */
  void calcTransformations() override;
  /**
   * @brief transform single point with rubber-sheeting in 3D
   *
   * @param[in] pt                  - Eigen::Vector3d:
   *                                  point to transform
   */
  void transformPoint(Eigen::Vector3d & pt) const override;
  /**
   * @brief get edges of tetrahedrons
   *
   * @param[out]                    - std::vector<std::vector<ProjPoint>>:
   *                                  edges of tetrahedrons
   */
  std::vector<std::vector<ProjPoint>> getEdges() const override;
  /**
   * @brief get vertices of tetrahedrons
   *
   * @param[out]                    - std::vector<std::vector<ProjPoint>>:
   *                                  vertices of tetrahedrons
   */
  std::vector<std::vector<ProjPoint>> getVertices() const override;

private:
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
 * @param[in] target_pt           - std::vector<ProjPoint>:
 *                                  target points
 * @param[in] source_pt           - std::vector<ProjPoint>:
 *                                  source points
 * @param[in] cps                 - std::vector<ControlPoint>:
 *                                  control points
 */
void Delaunay::corner2cps(
  const std::vector<Eigen::Vector3d> & target_pt, const std::vector<Eigen::Vector3d> & src_pt,
  std::vector<ControlPoint> & cps) const
{
  for (size_t i = 0; i < target_pt.size(); ++i) {
    ControlPoint P(
      src_pt.at(i)(0), src_pt.at(i)(1), src_pt.at(i)(2), target_pt.at(i)(0), target_pt.at(i)(1),
      target_pt.at(i)(2));
    cps.push_back(P);
  }
}
/**
 * @brief calculate enclosing rectangle controlpoints from selected ones and trajectory
 *
 * @param[in] size                - std::vector<double>:
 *                                  expansion (percentage) -> x,y
 * @param[in] target              - std::vector<ProjPoint>:
 *                                  target trajectory
 * @param[in] cps                 - std::vector<ControlPoint>:
 *                                  control points on trajectory
 */
void Delaunay_2D::enclosingControlPoints(
  const std::vector<double> & size, const std::vector<ProjPoint> & target,
  std::vector<ControlPoint> & cps) const
{
  if (size.size() != 2) {
    std::cout << "Wrong size of boundary definitions!!" << std::endl;
  }
  // Get maximum x, y, and z values from target trajectory
  std::vector<double> x, y;

  for (const auto & pt : target) {
    x.push_back(pt.pos_(0));
    y.push_back(pt.pos_(1));
  }

  double min_x = *std::min_element(x.begin(), x.end());
  double max_x = *std::max_element(x.begin(), x.end());
  double min_y = *std::min_element(y.begin(), y.end());
  double max_y = *std::max_element(y.begin(), y.end());

  // Set size of cube
  double dx, dy;
  dx = size.at(0) * std::abs(max_x - min_x);
  dy = size.at(1) * std::abs(max_y - min_y);

  // Define cube
  Eigen::Vector3d v1(min_x - dx, min_y - dy, 0.0);
  Eigen::Vector3d v2(min_x - dx, max_y + dy, 0.0);
  Eigen::Vector3d v3(max_x + dx, max_y + dy, 0.0);
  Eigen::Vector3d v4(max_x + dx, min_y - dy, 0.0);

  std::vector<Eigen::Vector3d> pts_target{v1, v2, v3, v4};

  // Calculate points for source
  std::vector<Eigen::Vector3d> pts_src;

  for (size_t i = 0; i < pts_target.size(); i++) {
    std::vector<double> vx, vy, d;

    // Iterate through controlpoints to find closest one
    for (auto & cp : cps) {
      vx.push_back(pts_target.at(i)(0) + cp.get_source_point()(0) - cp.get_target_point()(0));
      vy.push_back(pts_target.at(i)(1) + cp.get_source_point()(1) - cp.get_target_point()(1));
      Eigen::Vector3d pt(vx.back(), vy.back(), 0.0);

      d.push_back(std::abs((pt - cp.get_source_point()).norm()));
    }
    const int ind = std::distance(d.begin(), std::min_element(d.begin(), d.end()));
    Eigen::Vector3d pt_src(vx.at(ind), vy.at(ind), 0.0);
    pts_src.push_back(pt_src);
  }

  // Add points to vector of control points
  this->corner2cps(pts_target, pts_src, cps);
}
/**
 * @brief insert point as 2D point into triangulation
 *
 * @param[in] pt                  - Eigen::Vector3d:
 *                                  point to insert
 */
void Delaunay_2D::insertPoint(const Eigen::Vector3d & pt)
{
  this->triangulation_.insert(Point2D(pt(0), pt(1)));
}
/**
 * @brief create mapping between control points and triangles
 *
 * @param[in] cps                 - std::vector<ControlPoint>:
 *                                  control points
 */
void Delaunay_2D::mapControlPoints(const std::vector<ControlPoint> & cps)
{
  // Iterate over every face handle (triangle) and save corresponding control points in struct
  for (DT2::Face_handle face : this->triangulation_.finite_face_handles()) {
    std::vector<ControlPoint> found_cps;
    // find 4 vertices
    for (size_t i = 0; i < 3; i++) {
      // Check every Controlpoint
      for (const auto & cp : cps) {
        Point2D p = this->triangulation_.triangle(face)[i];
        if (p.x() == cp.get_target_point()(0) && p.y() == cp.get_target_point()(1)) {
          found_cps.push_back(cp);
        }
      }
    }
    if (found_cps.size() != 3)
      std::cout << "\033[1;32m~~~~~~~~~~> Error while mapping CPs to triangulation !\033[0m"
                << std::endl;

    this->mapping_.push_back(CpsMap_2D(found_cps, face));
  }
}
/**
 * @brief calculate 3x3 transformation matrices
 */
void Delaunay_2D::calcTransformations()
{
  std::vector<Eigen::Vector3d> target;
  std::vector<Eigen::Vector3d> source;

  for (CpsMap_2D entry : this->mapping_) {
    // Clean up vector
    target.clear();
    source.clear();
    for (size_t i = 0; i < 3; i++) {
      target.push_back(entry.get_cps().at(i).get_target_point());
      source.push_back(entry.get_cps().at(i).get_source_point());
    }

    // Construct linear equations and solve to find transformation matrix for tetrahedron
    Eigen::Matrix3d mat = solve_linear_2d(source, target);
    this->mat_.push_back(mat);
  }
}
/**
 * @brief transform single point with rubber-sheeting in 2D
 *
 * @param[in] pt                  - Eigen::Vector3d:
 *                                  point to transform
 */
void Delaunay_2D::transformPoint(Eigen::Vector3d & pt) const
{
  // Find area the point is in
  FaceHandle face = this->triangulation_.locate(Point2D(pt(0), pt(1)));
  auto map = find_if(this->mapping_.begin(), this->mapping_.end(), [face](const CpsMap_2D & map) {
    return face == map.get_face_handle();
  });
  int index = std::distance(this->mapping_.begin(), map);
  if (this->triangulation_.is_infinite(face) || index >= static_cast<int>(this->mapping_.size())) {
    // Point outside triangulation
    pt = Eigen::Vector3d(0.0, 0.0, 0.0);
  } else {
    // Point inside triangulation
    const Eigen::Vector3d point(pt(0), pt(1), 1.0);
    const Eigen::Vector3d pt_rs = this->mat_.at(index) * point;
    pt = Eigen::Vector3d(pt_rs(0), pt_rs(1), pt(2));
  }
}
/**
 * @brief get edges of triangles
 *
 * @param[out]                    - std::vector<std::vector<ProjPoint>>:
 *                                  edges of triangles
 */
std::vector<std::vector<ProjPoint>> Delaunay_2D::getEdges() const
{
  // Create linestrings and linestring array
  std::vector<std::vector<ProjPoint>> edges{};

  for (const CpsMap_2D & entry : this->mapping_) {
    const FaceHandle face = entry.get_face_handle();
    // Create points for linestring
    ProjPoint p1(face->vertex(0)->point().x(), face->vertex(0)->point().y(), 0.0, 0.0, 0.0, 0.0);
    ProjPoint p2(face->vertex(1)->point().x(), face->vertex(1)->point().y(), 0.0, 0.0, 0.0, 0.0);
    ProjPoint p3(face->vertex(2)->point().x(), face->vertex(2)->point().y(), 0.0, 0.0, 0.0, 0.0);

    std::vector<ProjPoint> ls1{p1, p2};
    std::vector<ProjPoint> ls2{p1, p3};
    std::vector<ProjPoint> ls3{p2, p3};

    edges.push_back(ls1);
    edges.push_back(ls2);
    edges.push_back(ls3);
  }
  return edges;
}
/**
 * @brief get vertices of triangles
 *
 * @param[out]                    - std::vector<std::vector<ProjPoint>>:
 *                                  vertices of triangles
 */
std::vector<std::vector<ProjPoint>> Delaunay_2D::getVertices() const
{
  std::vector<std::vector<ProjPoint>> vertices;
  std::vector<ProjPoint> tet;

  for (const CpsMap_2D & entry : this->mapping_) {
    tet.clear();
    for (size_t i = 0; i < 3; ++i) {
      const Eigen::Vector3d p = entry.get_cps().at(i).get_target_point();
      ProjPoint pt(p(0), p(1), 0.0, 0.0, 0.0, 0.0);
      tet.push_back(pt);
    }
    vertices.push_back(tet);
  }
  return vertices;
}
/**
 * @brief solve linear equations for triangle
 *
 * @param[in] src                 - std::vector<Eigen::Vector3d>:
 *                                  source points
 * @param[in] target              - std::vector<Eigen::Vector3d>:
 *                                  target points
 * @param[out]                    - Eigen::Matrix3d:
 *                                  transformation matrix for triangle
 */
Eigen::Matrix3d Delaunay_2D::solve_linear_2d(
  const std::vector<Eigen::Vector3d> & src, const std::vector<Eigen::Vector3d> & target)
{
  // Construct linear equations
  // b
  Eigen::VectorXd b(9);
  b << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  int i = 0;

  for (const auto & pt : src) {
    b(3 * i) = pt(0);
    b(3 * i + 1) = pt(1);
    ++i;
  }
  // A
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(9, 9);
  int j = 0;
  for (const auto & pt : target) {
    A.block(3 * j + 0, 0, 1, 3) << pt(0), pt(1), 1.0;
    A.block(3 * j + 1, 3, 1, 3) << pt(0), pt(1), 1.0;
    A.block(3 * j + 2, 6, 1, 3) << pt(0), pt(1), 1.0;
    ++j;
  }
  // solve
  const Eigen::VectorXd ti = A.fullPivHouseholderQr().solve(b);

  // Reshape to get matrix
  Eigen::Matrix3d trans;
  trans << ti(0), ti(1), ti(2), ti(3), ti(4), ti(5), ti(6), ti(7), ti(8);
  return trans;
}
/**
 * @brief calculate enclosing cube controlpoints from selected ones and trajectory
 *
 * @param[in] size                - std::vector<double>:
 *                                  expansion (percentage) -> x,y,z
 * @param[in] target              - std::vector<ProjPoint>:
 *                                  target trajectory
 * @param[in] cps                 - std::vector<ControlPoint>:
 *                                  control points on trajectory
 */
void Delaunay_3D::enclosingControlPoints(
  const std::vector<double> & size, const std::vector<ProjPoint> & target,
  std::vector<ControlPoint> & cps) const
{
  if (size.size() != 3) {
    std::cout << "Wrong size of boundary definitions!!" << std::endl;
  }
  // Get maximum x, y, and z values from target trajectory
  std::vector<double> x, y, z;

  for (const auto & pt : target) {
    x.push_back(pt.pos_(0));
    y.push_back(pt.pos_(1));
    z.push_back(pt.pos_(2));
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
      vx.push_back(pts_target.at(i)(0) + cp.get_source_point()(0) - cp.get_target_point()(0));
      vy.push_back(pts_target.at(i)(1) + cp.get_source_point()(1) - cp.get_target_point()(1));
      vz.push_back(pts_target.at(i)(2) + cp.get_source_point()(2) - cp.get_target_point()(2));
      Eigen::Vector3d pt(vx.back(), vy.back(), vz.back());

      d.push_back(std::abs((pt - cp.get_source_point()).norm()));
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
void Delaunay_3D::insertPoint(const Eigen::Vector3d & pt)
{
  this->triangulation_.insert(Point3D(pt(0), pt(1), pt(2)));
}
/**
 * @brief create mapping between control points and tetrahedrons
 *
 * @param[in] cps                 - std::vector<ControlPoint>:
 *                                  control points
 */
void Delaunay_3D::mapControlPoints(const std::vector<ControlPoint> & cps)
{
  // Iterate over every cell (tetrahedron) and save corresponding control points in struct
  for (DT3::Cell_handle tetra : this->triangulation_.finite_cell_handles()) {
    std::vector<ControlPoint> found_cps;
    // find 4 vertices
    for (size_t i = 0; i < 4; i++) {
      // Check every Controlpoint
      for (auto cp : cps) {
        Point3D p = this->triangulation_.tetrahedron(tetra)[i];
        if (
          p.x() == cp.get_target_point()(0) && p.y() == cp.get_target_point()(1) &&
          p.z() == cp.get_target_point()(2)) {
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
void Delaunay_3D::calcTransformations()
{
  std::vector<Eigen::Vector3d> target;
  std::vector<Eigen::Vector3d> source;

  for (CpsMap_3D entry : this->mapping_) {
    // Clean up vector
    target.clear();
    source.clear();
    for (size_t i = 0; i < 4; i++) {
      target.push_back(entry.get_cps().at(i).get_target_point());
      source.push_back(entry.get_cps().at(i).get_source_point());
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
void Delaunay_3D::transformPoint(Eigen::Vector3d & pt) const
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
 * @brief get edges of tetrahedrons
 *
 * @param[out]                    - std::vector<std::vector<ProjPoint>>:
 *                                  edges of tetrahedrons
 */
std::vector<std::vector<ProjPoint>> Delaunay_3D::getEdges() const
{
  // Create linestrings and linestring array
  std::vector<std::vector<ProjPoint>> tet{};

  for (const CpsMap_3D & entry : this->mapping_) {
    CellHandle cell = entry.get_cell_handle();
    // Create points for linestring
    ProjPoint p1(
      cell->vertex(0)->point().x(), cell->vertex(0)->point().y(), cell->vertex(0)->point().z(), 0.0,
      0.0, 0.0);
    ProjPoint p2(
      cell->vertex(1)->point().x(), cell->vertex(1)->point().y(), cell->vertex(1)->point().z(), 0.0,
      0.0, 0.0);
    ProjPoint p3(
      cell->vertex(2)->point().x(), cell->vertex(2)->point().y(), cell->vertex(2)->point().z(), 0.0,
      0.0, 0.0);
    ProjPoint p4(
      cell->vertex(3)->point().x(), cell->vertex(3)->point().y(), cell->vertex(3)->point().z(), 0.0,
      0.0, 0.0);

    std::vector<ProjPoint> ls1{p1, p2};
    std::vector<ProjPoint> ls2{p1, p3};
    std::vector<ProjPoint> ls3{p1, p4};
    std::vector<ProjPoint> ls4{p2, p3};
    std::vector<ProjPoint> ls5{p2, p4};
    std::vector<ProjPoint> ls6{p3, p4};

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
 * @param[out]                    - std::vector<std::vector<ProjPoint>>:
 *                                  vertices of tetrahedrons
 */
std::vector<std::vector<ProjPoint>> Delaunay_3D::getVertices() const
{
  std::vector<std::vector<ProjPoint>> vertices;
  std::vector<ProjPoint> tet;

  for (const CpsMap_3D & entry : this->mapping_) {
    tet.clear();
    for (size_t i = 0; i < 4; ++i) {
      const Eigen::Vector3d p = entry.get_cps().at(i).get_target_point();
      ProjPoint pt(p(0), p(1), p(2), 0.0, 0.0, 0.0);
      tet.push_back(pt);
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
Eigen::Matrix4d Delaunay_3D::solve_linear_3d(
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
