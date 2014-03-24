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

#ifndef THEIA_VISION_SFM_CAMERA_CAMERA_POSE_H_
#define THEIA_VISION_SFM_CAMERA_CAMERA_POSE_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "theia/alignment/alignment.h"

namespace theia {

// This class contains the full camera pose including extrinsic parameters as
// well as intrinsic parameters. Extrinsic parameters include the camera
// orientation and position, and intrinsic parameters include focal length and
// (up to 4-parameter) radial distortion. Methods are provided for common
// transformations and projections. We define several types of transformations:
//
//   world-to-camera: a transformation that maps 3D world coordinates to
//      coordinates in the camera coordinate system. This is _not_ a projection.
//   camera-to-image: a projection that maps points in the camera coordinate
//      system onto the image plane given the intrinsic parameters of the
//      camera.
//   world-to-image: projects world points on to the image plane given the
//      extrinsic pose and the camera intrinsics.
//
// The inverse transformations are also provided where they make sense.
class CameraPose {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraPose();
  ~CameraPose();

  // ------------------------ Initialization methods ------------------------ //

  // Initialize the camera pose with the full extrinsic (rotation and
  // translation) and intrinsic (calibration matrix and radial distortion)
  // parameters. The extrinsic parameters should provide world-to-camera
  // transformations.
  void InitializePose(const Eigen::Matrix3d& rotation,
                      const Eigen::Vector3d& translation,
                      const Eigen::Matrix3d& calibration, const double k1,
                      const double k2, const double k3, const double k4);

  // Initialize the pose with a projection matrix given by P = K * [R | t],
  // where K is the calibration matrix, R is the rotation matrix, and t is the
  // translation. The projection matrix provided should be a world-to-image
  // transformation (as opposed to world-to-camera).
  void InitializePose(const Eigen::Matrix<double, 3, 4>& projection_matrix,
                      const double k1, const double k2, const double k3,
                      const double k4);

  // Initialize the pose with a given transformation matrix that defines the
  // world-to-camera transformation.
  void InitializePose(const Eigen::Matrix<double, 3, 4>& transformation_matrix,
                      const Eigen::Matrix3d& calibration, const double k1,
                      const double k2, const double k3, const double k4);

  void InitializePose(const CameraPose& pose);

  // ---------------------------- Getter methods ---------------------------- //

  // Get the rotation componenet of the transformation matrix.
  Eigen::Matrix3d rotation_matrix() const {
    return transformation_matrix_.rotation();
  }

  // Get the translation componenet of the transformation matrix.
  Eigen::Vector3d translation() const {
    return transformation_matrix_.translation();
  }

  // Get the camera position in the world coordinate system defined as
  // position = -R' * t.
  Eigen::Vector3d position() const {
    return transformation_matrix_.inverse().translation();
  }

  // Get the 3x3 camera calibration matrix defined by K = diag(f, f, 1).
  Eigen::Matrix3d calibration_matrix() const { return calibration_matrix_; }

  // Get the focal length of the camera.
  double focal_length() const {
    return calibration_matrix_(0, 0) / calibration_matrix_(2, 2);
  }

  // Returns the radial distortion parameters.
  void radial_distortion(double* k1, double* k2, double* k3, double* k4) const {
    *k1 = k1_;
    *k2 = k2_;
    *k3 = k3_;
    *k4 = k4_;
  }

  Eigen::Matrix<double, 3, 4> projection_matrix() const {
    return projection_matrix_;
  }

  Eigen::Matrix<double, 3, 4> transformation_matrix() const {
    return transformation_matrix_.matrix();
  }

  // ------------------------ Transformation methods ------------------------ //

  // Transforms a point from the world coordinate system to the camera
  // coordinate system.
  void WorldToCamera(const Eigen::Vector3d& world_point,
                     Eigen::Vector3d* camera_point) const;
  // Transformation method for multiple points.
  void WorldToCamera(const std::vector<Eigen::Vector3d>& world_point,
                     std::vector<Eigen::Vector3d>* camera_point) const;

  // Transforms a point from the camera coordinate system to the world
  // coordinate system.
  void CameraToWorld(const Eigen::Vector3d& camera_point,
                     Eigen::Vector3d* world_point) const;
  // Transformation method for multiple points.
  void CameraToWorld(const std::vector<Eigen::Vector3d>& camera_point,
                     std::vector<Eigen::Vector3d>* world_point) const;

  // Projects the 3D points in camera coordinates into the image plane using the
  // calibration matrix of the camera.
  void CameraToImage(const Eigen::Vector3d& camera_point,
                     Eigen::Vector2d* image_point) const;
  // Projection method for multiple points. NOTE: this method is void, and does
  // not indicate whether points are in front of behind the camera.
  void CameraToImage(const std::vector<Eigen::Vector3d>& camera_point,
                     std::vector<Eigen::Vector2d>* image_point) const;

  // Projects the 3D points in world coordinates into the image plane using the
  // projection matrix of the camera. Returns true if the point is in front of
  // the camera and false otherwise.
  bool WorldToImage(const Eigen::Vector3d& world_point,
                    Eigen::Vector2d* image_point) const;
  // Projection method for multiple points. NOTE: this method is void, and does
  // not indicate whether points are in front of behind the camera.
  void WorldToImage(const std::vector<Eigen::Vector3d>& world_point,
                    std::vector<Eigen::Vector2d>* image_point) const;

  // Undistorts the image point using the radial distortion parameters.
  void UndistortImagePoint(const Eigen::Vector2d& distorted_point,
                           Eigen::Vector2d* undistorted_point) const;
  // Undistort multiple points at the same time.
  void UndistortImagePoint(
      const std::vector<Eigen::Vector2d>& distorted_point,
      std::vector<Eigen::Vector2d>* undistorted_point) const;

 private:
  // The internal calibration of the camera. K_ has the form diag(f, f, 1),
  // where f is the focal length of the camera.
  Eigen::Matrix3d calibration_matrix_;

  // The 1st radial distortion parameter.
  double k1_;
  // The 2nd radial distortion parameter.
  double k2_;
  // The 3rd radial distortion parameter.
  double k3_;
  // The 4th radial distortion parameter.
  double k4_;

  // The projection matrix P = K * [R | t] projecting points from the global
  // coordinate system to the local coordinate system.
  Eigen::Matrix<double, 3, 4> projection_matrix_;

  // The transformation matrix T = [R | t] transforming points from the global
  // coordinate system to the local coordinate system. NOTE: Eigen enforces some
  // constraints on this matrix to make it actually be affine. For instance, the
  // determinant of the rotation will always be set to 1.
  Eigen::AffineCompact3d transformation_matrix_;
};

}  // namespace theia

#endif  // THEIA_VISION_SFM_CAMERA_CAMERA_POSE_H_
