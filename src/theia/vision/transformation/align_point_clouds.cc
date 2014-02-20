// Copyright (C) 2013 The Regents of the University of California (Regents).
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

#include "theia/vision/transformation/align_point_clouds.h"

#include <glog/logging.h>
#include <Eigen/Dense>

// Use Besel-McKay registration to align point clouds to a model. We use SVD
// decomposition because it is better at finding the absolute minimum error for
// rotation as compared to a true ICP that Besl-McKay describe, which is only
// guaranteed to find a local minima.
namespace theia {

void AlignPointCloudsICP(const int num_points,
                         const double left[],
                         const double right[],
                         double rotation[3 * 3],
                         double translation[3]) {
  Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic> > left_points(
      left, 3, num_points);
  Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic> > right_points(
      right, 3, num_points);

  Eigen::Vector3d left_centroid = left_points.rowwise().mean();
  Eigen::Vector3d right_centroid = right_points.rowwise().mean();

  // Calculate cross correlation matrix based on the points shifted about the
  // centroid.
  Eigen::Matrix3d cross_correlation = Eigen::Matrix3d::Zero();
  for (int i = 0; i < num_points; i++) {
    cross_correlation += (left_points.col(i) - left_centroid) *
                         (right_points.col(i) - right_centroid).transpose();
  }

  // Compute SVD decomposition of the cross correlation. This is excessive for 3
  // dimensions, but it is more stable than ICP.  TODO(cmsweeney): Evaluate the
  // performance more (SVD method runs in roughly 0.15ms on some tests
  // performed) to determine whether the closed form solution (see Horn's
  // quaternion method for analytic computation of rotation using quaternions)
  // is preferred.
  Eigen::JacobiSVD<Eigen::Matrix3d>
      svd(cross_correlation, Eigen::ComputeFullU | Eigen::ComputeFullV);
  // Use the memory allocated from the rotation array passed in to specify the
  // memory space for the rotation matrix calculated. This is both memory
  // efficient and prevents any memory scope errors (same concept used for
  // translation below).
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > rotation_mat(
      rotation);
  rotation_mat = svd.matrixV() * (svd.matrixU().transpose());

  // Rotation is solved for, but the optimal translation is the difference of
  // the right centroid and the rotated left centroid.
  Eigen::Vector3d rotated_left_centroid = rotation_mat * left_centroid;
  Eigen::Map<Eigen::RowVector3d> translation_vec(translation);
  translation_vec = right_centroid - rotated_left_centroid;
}

void AlignPointCloudsUmeyama(const int num_points,
                             const double left[],
                             const double right[],
                             double rotation[3 * 3],
                             double translation[3],
                             double* scale) {
  // First, compute the centroids.
  Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic> > left_points(
      left, 3, num_points);
  Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic> > right_points(
      right, 3, num_points);

  // Get the transformation via Umeyama's least squares algorithm. This returns
  // a matrix of the form:
  //    [ s * R  t]
  //    [   0    1]
  // from which we can extract the scale, rotation, and translation.
  const Eigen::Matrix4d transformation_mat =
      Eigen::umeyama(left_points, right_points, true);

  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > rotation_mat(
      rotation);
  rotation_mat = transformation_mat.topLeftCorner<3, 3>();
  *scale = pow(rotation_mat.determinant(), 1.0 / 3.0);
  rotation_mat /= *scale;
  Eigen::Map<Eigen::Vector3d> translation_vec(translation);
  translation_vec = transformation_mat.topRightCorner<3, 1>();
}

}  // namespace theia
