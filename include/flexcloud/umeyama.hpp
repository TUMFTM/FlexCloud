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
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <string>
#include <vector>

#include "utility.hpp"
namespace flexcloud
{
/**
 * @brief base class for Umeyama transformation
 */
class Umeyama
{
public:
  // Destructor
  ~Umeyama() = default;
  /**
   * @brief insert source vector of transformation
   *
   * @param[in] src                 - std::vector<Eigen::Vector3d>:
   *                                  source trajectory
   */
  void insertSource(const std::vector<Eigen::Vector3d> & src);
  /**
   * @brief insert target vector of transformation
   *
   * @param[in] target              - std::vector<Eigen::Vector3d>:
   *                                  target trajectory
   */
  void insertTarget(const std::vector<Eigen::Vector3d> & target);
  /**
   * @brief calculate rigid transformation matrix
   */
  void calcTransform();
  /**
   * @brief transform point with transformation
   *
   * @param[in] pt                  - Eigen::Isometry3d:
   *                                  point to transform
   */
  void transformPoint(Eigen::Isometry3d & pt) const;
  /**
   * @brief transform point with transformation
   *
   * @param[in] pt                  - Eigen::Vector3d:
   *                                  point to transform
   */
  void transformPoint(Eigen::Vector3d & pt) const;

protected:
  /**
   * @brief convert trajectory to 3xX-matrix
   *
   * @param[in] ls                  - std::vector<Eigen::Vector3d>
   *                                  trajectory
   * @param[out]                    - Eigen::MatrixXd:
   *                                  3xX matrix
   */
  Eigen::MatrixXd ls2mat(const std::vector<Eigen::Vector3d> & ls);
  // 4x4 transformation matrix for 3D transformation
  Eigen::Matrix4d trans_;
  // class variables representing trajectories to align
  std::vector<Eigen::Vector3d> src_;
  std::vector<Eigen::Vector3d> target_;
};
/**
 * @brief insert source vector of transformation
 *
 * @param[in] src                 - std::vector<Eigen::Vector3d>:
 *                                  source trajectory
 */
void Umeyama::insertSource(const std::vector<Eigen::Vector3d> & src) { this->src_ = src; }
/**
 * @brief insert target vector of transformation
 *
 * @param[in] target              - std::vector<Eigen::Vector3d>:
 *                                  target trajectory
 */
void Umeyama::insertTarget(const std::vector<Eigen::Vector3d> & target) { this->target_ = target; }
/**
 * @brief calculate rigid transformation matrix in 3D
 */
void Umeyama::calcTransform()
{
  // Convert to eigen-matrices to use eigen-functions
  const Eigen::MatrixXd src_mat = this->ls2mat(this->src_);
  const Eigen::MatrixXd target_mat = this->ls2mat(target_);
  if (src_mat.cols() != target_mat.cols()) {
    std::cout << "\033[1;31m!! Trajectories have different sizes !!\033[0m" << std::endl;
  }

  // Umeyama Transformation without scaling
  const Eigen::Matrix4d trans = Eigen::umeyama(src_mat, target_mat, false);

  // Calculate scaling factor between poses and GPS trajectory
  // Apply Umeyama-algorithm with scaling => R_scal = c * R
  const Eigen::MatrixXd trans_scaling = Eigen::umeyama(src_mat, target_mat);
  const double scale = trans_scaling(0, 0) / trans(0, 0);

  if (scale < 0.95) {
    std::cout << "\033[1;31m!! High scaling factor between poses and GPS data. "
              << "Are you sure they belong together? !!\033[0m" << std::endl;
  }
  this->trans_ = trans.inverse();
}
/**
 * @brief transform point with 3D transformation
 *
 * @param[in] pt                  - Eigen::Isometry3d
 *                                  point to transform
 */
void Umeyama::transformPoint(Eigen::Isometry3d & pt) const
{
  // Point inside triangulation
  // const Eigen::Vector4d point(pt(0), pt(1), pt(2), 1.0);
  // const Eigen::Vector4d pt_al = this->trans_ * point;
  pt.matrix() = this->trans_ * pt.matrix();
}
/**
 * @brief transform point with 3D transformation
 *
 * @param[in] pt                  - Eigen::Vector3d
 *                                  point to transform
 */
void Umeyama::transformPoint(Eigen::Vector3d & pt) const
{
  // Point inside triangulation
  const Eigen::Vector4d point(pt(0), pt(1), pt(2), 1.0);
  const Eigen::Vector4d pt_al = this->trans_ * point;
  pt = pt_al.head<3>();
}
/**
 * @brief convert trajectory to 3xX-matrix
 *
 * @param[in] ls                  - std::vector<Eigen::Vector3d>
 *                                  trajectory
 * @param[out]                    - Eigen::MatrixXd:
 *                                  3xX matrix
 */
Eigen::MatrixXd Umeyama::ls2mat(const std::vector<Eigen::Vector3d> & ls)
{
  // Initialize number of interpolation points and output matrix
  Eigen::MatrixXd mat(3, ls.size());

  for (size_t i = 0; i < ls.size(); ++i) {
    mat.col(i) << ls.at(i)(0), ls.at(i)(1), ls.at(i)(2);
  }
  return mat;
}
}  // namespace flexcloud
