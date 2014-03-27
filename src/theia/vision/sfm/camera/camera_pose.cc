// Copyright (C) 2014 The Regents of the University of California (Regents).
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)
//         Torsten Sattler (sattlert@inf.ethz.ch)

#include "theia/vision/sfm/camera/camera_pose.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include "theia/alignment/alignment.h"

namespace theia {

using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;

CameraPose::CameraPose() : k1_(0.0), k2_(0.0), k3_(0.0), k4_(0.0) {
  calibration_matrix_.setIdentity();
  transformation_matrix_.setIdentity();
  projection_matrix_.setIdentity();
}

CameraPose::~CameraPose() {}

void CameraPose::InitializePose(const Matrix3d& rotation,
                                const Vector3d& translation,
                                const Matrix3d& calibration, const double k1,
                                const double k2, const double k3,
                                const double k4) {
  transformation_matrix_.linear() = rotation;
  transformation_matrix_.translation() = translation;
  calibration_matrix_ = calibration;
  projection_matrix_ = calibration_matrix_ * transformation_matrix_.matrix();
  k1_ = k1;
  k2_ = k2;
  k3_ = k3;
  k4_ = k4;
}

// The projection matrix is defined such that P = K * [R | t], where K = diag(f,
// f, 1) for focal length f. The rows of the rotation matrix must have the same
// norm, so we can recover the focal length by comparing the norms of the first
// and third projection matrix row (since only one of them will be affected by
// the focal length).
void CameraPose::InitializePose(const Matrix<double, 3, 4>& projection_matrix,
                                const double k1, const double k2,
                                const double k3, const double k4) {
  projection_matrix_ = projection_matrix;
  k1_ = k1;
  k2_ = k2;
  k3_ = k3;
  k4_ = k4;

  // Recover focal length and construct the calibration matrix. NOTE: setting
  // the calibration matrix to diag(1, 1, f) is OK since we only recover the
  // focal length as the ratio of diagonal elements (see focal_length() method).
  const double inv_focal_length = sqrt(projection_matrix_.block<1, 3>(
      2, 0).squaredNorm() / projection_matrix.block<1, 3>(0, 0).squaredNorm());
  calibration_matrix_ =
      Eigen::DiagonalMatrix<double, 3>(1.0, 1.0, inv_focal_length);

  // Since P = K * T, where T = [R | t], we can recover T with T = K^-1 * P. We
  // must use a temp matrix so that the AffineCompact3d type of Eigen does not
  // fix the det of the rotation. We use the det of the rotation to recover the
  // sign of the focal length.
  const Eigen::Matrix<double, 3, 4> temp_transformation =
      calibration_matrix_.inverse() * projection_matrix_;

  // Recover the sign of the focal length using the fact that a rotation matrix
  // should have a determinant of 1.
  double det = temp_transformation.block<3, 3>(0, 0).determinant();
  if (det < 0.0) {
    calibration_matrix_(2, 2) *= -1.0;
  }
  transformation_matrix_ = calibration_matrix_.inverse() * projection_matrix_;
}

void CameraPose::InitializePose(
    const Matrix<double, 3, 4>& transformation_matrix,
    const Matrix3d& calibration, const double k1, const double k2,
    const double k3, const double k4) {
  transformation_matrix_ = transformation_matrix;
  calibration_matrix_ = calibration;
  projection_matrix_ = calibration_matrix_ * transformation_matrix_.matrix();
  k1_ = k1;
  k2_ = k2;
  k3_ = k3;
  k4_ = k4;
}

void CameraPose::InitializePose(const CameraPose& pose) {
  InitializePose(pose.transformation_matrix_.matrix(), pose.calibration_matrix_,
                 pose.k1_, pose.k2_, pose.k3_, pose.k4_);
}

void CameraPose::WorldToCamera(const Vector3d& world_point,
                               Vector3d* camera_point) const {
  *camera_point = transformation_matrix_ * world_point;
}

void CameraPose::WorldToCamera(const std::vector<Vector3d>& world_point,
                               std::vector<Vector3d>* camera_point) const {
  Map<const Matrix<double, 3, Eigen::Dynamic> > world_point_matrix(
      world_point[0].data(), 3, world_point.size());
  camera_point->clear();
  camera_point->resize(world_point.size());
  Map<Matrix<double, 3, Eigen::Dynamic> > camera_point_matrix(
      (*camera_point)[0].data(), 3, camera_point->size());
  camera_point_matrix = transformation_matrix_ * world_point_matrix;
}

void CameraPose::CameraToWorld(const Vector3d& camera_point,
                               Vector3d* world_point) const {
  *world_point = transformation_matrix_.inverse() * camera_point;
}

void CameraPose::CameraToWorld(const std::vector<Vector3d>& camera_point,
                               std::vector<Vector3d>* world_point) const {
  Map<const Matrix<double, 3, Eigen::Dynamic> > camera_point_matrix(
      camera_point[0].data(), 3, camera_point.size());
  world_point->clear();
  world_point->resize(camera_point.size());
  Map<Matrix<double, 3, Eigen::Dynamic> > world_point_matrix(
      (*world_point)[0].data(), 3, world_point->size());
  world_point_matrix = transformation_matrix_.inverse() * camera_point_matrix;
}

void CameraPose::CameraToImage(const Vector3d& camera_point,
                               Vector2d* image_point) const {
  *image_point = (calibration_matrix_ * camera_point).hnormalized();
}

void CameraPose::CameraToImage(const std::vector<Vector3d>& camera_point,
                               std::vector<Vector2d>* image_point) const {
  Map<const Matrix<double, 3, Eigen::Dynamic> > camera_point_matrix(
      camera_point[0].data(), 3, camera_point.size());
  image_point->clear();
  image_point->resize(camera_point.size());
  Map<Matrix<double, 2, Eigen::Dynamic> > image_point_matrix(
      (*image_point)[0].data(), 2, image_point->size());
  image_point_matrix = (calibration_matrix_.inverse() * camera_point_matrix)
      .colwise().hnormalized();
}

bool CameraPose::WorldToImage(const Vector3d& world_point,
                              Vector2d* image_point) const {
  const Vector3d proj_point = projection_matrix_ * world_point.homogeneous();
  // Return false if the point is behind the camera.
  if (calibration_matrix_(0, 0) * proj_point[2] <= 0.0) {
    return false;
  }
  *image_point = proj_point.hnormalized();
  return true;
}

void CameraPose::WorldToImage(const std::vector<Vector3d>& world_point,
                              std::vector<Vector2d>* image_point) const {
  Map<const Matrix<double, 3, Eigen::Dynamic> > world_point_matrix(
      world_point[0].data(), 3, world_point.size());

  // Create a temp 3xN matrix of the projected points... eigen tends to struggle
  // when using colwise().homogeneous() and colwise().hnormalized() in the same
  // expression.
  Matrix<double, 3, Eigen::Dynamic> temp_proj_mat =
      projection_matrix_ * (world_point_matrix.colwise().homogeneous());

  image_point->clear();
  image_point->resize(world_point.size());
  Map<Matrix<double, 2, Eigen::Dynamic> > image_point_matrix(
      (*image_point)[0].data(), 2, image_point->size());
  image_point_matrix = temp_proj_mat.colwise().hnormalized();
}

void CameraPose::UndistortImagePoint(const Vector2d& distorted_point,
                                     Vector2d* undistorted_point) const {
  const double r = distorted_point.squaredNorm();
  const double w_term =
      1.0 + k1_ * r + k2_ * r * r + k3_ * r * r * r + k4_ * r * r * r * r;
  *undistorted_point = distorted_point / w_term;
}

void CameraPose::UndistortImagePoint(
    const std::vector<Vector2d>& distorted_point,
    std::vector<Vector2d>* undistorted_point) const {
  undistorted_point->clear();
  undistorted_point->resize(distorted_point.size());

  Eigen::Map<const Eigen::Matrix<double, 2, Eigen::Dynamic> > distorted_map(
      distorted_point[0].data(), 2, distorted_point.size());
  Eigen::ArrayXd radius = distorted_map.colwise().squaredNorm();
  radius =
      1.0 + radius * k1_ + radius * radius * k2_ +
      radius * radius * radius * k3_ + radius * radius * radius * radius * k4_;

  Eigen::Map<Eigen::Matrix<double, 2, Eigen::Dynamic> > undistorted_map(
      (*undistorted_point)[0].data(), 2, undistorted_point->size());
  undistorted_map = distorted_map;
  undistorted_map = undistorted_map.array().rowwise() / radius.transpose();
}

}  // namespace theia
