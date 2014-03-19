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

#include "theia/vision/sfm/pose/eight_point_fundamental_matrix.h"

#include <ceres/ceres.h>
#include <glog/logging.h>
#include "Eigen/Core"
#include "Eigen/SVD"
#include "Eigen/LU"

#include "theia/vision/sfm/pose/util.h"
#include "theia/vision/sfm/projection_matrix.h"
#include "theia/vision/sfm/triangulation/triangulation.h"

namespace theia {
namespace {
using Eigen::JacobiSVD;
using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::Vector2d;
using Eigen::Vector3d;

// Computes the right epipole from a fundamental matrix.
void ComputeEpipoles(const Matrix3d& fundamental_matrix,
                     Vector3d* right_epipole) {
  JacobiSVD<Matrix3d> f_svd(fundamental_matrix, Eigen::ComputeFullU);
  *right_epipole = f_svd.matrixU().rightCols<1>();
}

// Reprojection error cost function based on reprojected from the currently
// estimated projection from image 1 to image 2. This projection matrix
// parameterizes the fundamental matrix.
struct FundamentalReprojectionError {
  FundamentalReprojectionError(const Vector2d& observed_1,
                               const Vector2d& observed_2)
      : observed_1(observed_1),
        observed_2(observed_2) {}

  template <typename T>
  bool operator()(const T* const projection_mat, const T* const world_point,
                  T* residuals) const {
    // Projection into the first image is the identity projection.
    T predicted_1[2];
    predicted_1[0] = world_point[0] / world_point[2];
    predicted_1[1] = world_point[1] / world_point[2];

    T predicted_2[2];
    T homogeneous_factor =
        world_point[0] * projection_mat[2] +
        world_point[1] * projection_mat[5] +
        world_point[2] * projection_mat[8] + projection_mat[11];

    predicted_2[0] = (world_point[0] * projection_mat[0] +
                      world_point[1] * projection_mat[3] +
                      world_point[2] * projection_mat[6] + projection_mat[9]) /
                     homogeneous_factor;
    predicted_2[1] = (world_point[0] * projection_mat[1] +
                      world_point[1] * projection_mat[4] +
                      world_point[2] * projection_mat[7] + projection_mat[10]) /
                     homogeneous_factor;

    residuals[0] = predicted_1[0] - T(observed_1[0]);
    residuals[1] = predicted_1[1] - T(observed_1[1]);
    residuals[2] = predicted_2[0] - T(observed_2[0]);
    residuals[3] = predicted_2[1] - T(observed_2[1]);

    return true;
  }

  Vector2d observed_1;
  Vector2d observed_2;
};

void NormalizedEightPointInternal(
    const std::vector<Vector2d>& image_1_points,
    const std::vector<Vector2d>& image_2_points,
    std::vector<Vector2d>* norm_img1_points,
    std::vector<Vector2d>* norm_img2_points,
    Matrix3d* img1_norm_mat,
    Matrix3d* img2_norm_mat,
    Matrix3d* fundamental_matrix) {
  // Normalize the image points.
  NormalizeImagePoints(image_1_points, norm_img1_points, img1_norm_mat);
  NormalizeImagePoints(image_2_points, norm_img2_points, img2_norm_mat);

  // Build the constraint matrix based on x2' * F * x1 = 0.
  Matrix<double, Eigen::Dynamic, 9> constraint_matrix(image_1_points.size(), 9);
  for (int i = 0; i < image_1_points.size(); i++) {
    constraint_matrix.block<1, 3>(i, 0) =
        (*norm_img1_points)[i].homogeneous();
    constraint_matrix.block<1, 3>(i, 0) *= (*norm_img2_points)[i].x();
    constraint_matrix.block<1, 3>(i, 3) =
        (*norm_img1_points)[i].homogeneous();
    constraint_matrix.block<1, 3>(i, 3) *= (*norm_img2_points)[i].y();
    constraint_matrix.block<1, 3>(i, 6) = (*norm_img1_points)[i].homogeneous();
  }

  // Solve the constraint equation for F from nullspace extraction.
  const Matrix<double, 9, 1> normalized_fvector =
      constraint_matrix.fullPivLu().kernel();

  // NOTE: This is the transpose of a valid fundamental matrix! We implement a
  // "lazy" transpose and defer it to the SVD a few lines below.
  Eigen::Map<const Matrix3d> normalized_fmatrix(normalized_fvector.data());

  // Find the closest singular matrix to F under frobenius norm. We can compute
  // this matrix with SVD.
  JacobiSVD<Matrix3d> fmatrix_svd(normalized_fmatrix.transpose(),
                                  Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vector3d singular_values = fmatrix_svd.singularValues();
  singular_values[2] = 0.0;
  *fundamental_matrix = fmatrix_svd.matrixU() * singular_values.asDiagonal() *
                        fmatrix_svd.matrixV().transpose();
}

}  // namespace

bool NormalizedEightPoint(const std::vector<Vector2d>& image_1_points,
                          const std::vector<Vector2d>& image_2_points,
                          Matrix3d* fundamental_matrix) {
  CHECK_EQ(image_1_points.size(), image_2_points.size());
  CHECK_GE(image_1_points.size(), 8);

  std::vector<Vector2d> norm_img1_points(image_1_points.size());
  std::vector<Vector2d> norm_img2_points(image_2_points.size());
  Matrix3d img1_norm_mat, img2_norm_mat, singular_fmatrix;
  NormalizedEightPointInternal(
      image_1_points, image_2_points, &norm_img1_points, &norm_img2_points,
      &img1_norm_mat, &img2_norm_mat, &singular_fmatrix);

  *fundamental_matrix =
      img2_norm_mat.transpose() * singular_fmatrix * img1_norm_mat;

  return true;
}

bool GoldStandardEightPoint(const std::vector<Vector2d>& image_1_points,
                            const std::vector<Vector2d>& image_2_points,
                            Matrix3d* fundamental_matrix) {
  CHECK_EQ(image_1_points.size(), image_2_points.size());
  CHECK_GE(image_1_points.size(), 8);

  // Convert image points to 2D points.
  std::vector<Vector2d> norm_img1_points(image_1_points.size());
  std::vector<Vector2d> norm_img2_points(image_2_points.size());

  Matrix3d img1_norm_mat, img2_norm_mat, singular_fmatrix;
  NormalizedEightPointInternal(
      image_1_points, image_2_points, &norm_img1_points, &norm_img2_points,
      &img1_norm_mat, &img2_norm_mat, &singular_fmatrix);

  // Construct a projection matrix from the epipoles (derived through SVD of the
  // candidate fundamental matrix).
  Vector3d epipole_right;
  ComputeEpipoles(singular_fmatrix, &epipole_right);
  const ProjectionMatrix identity_transformation =
      TransformationMatrix::Identity().matrix();
  ProjectionMatrix right_projection;
  right_projection.col(3) = epipole_right;
  right_projection.block<3, 3>(0, 0) =
      CrossProductMatrix(epipole_right) * singular_fmatrix;

  // Minimize reprojection error cost via Ceres.
  ceres::Problem problem;
  std::vector<Vector3d> world_points(image_1_points.size());
  for (int i = 0; i < image_1_points.size(); i++) {
    // Triangulate 3D point.
    CHECK(
        Triangulate(identity_transformation, right_projection,
                    norm_img1_points[i], norm_img2_points[i], &world_points[i]))
        << "Could not triangulate the point for the fundamental matrix "
           "optimizaiton.";

    // Add residuals and parameters to the minimizer problem.
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<FundamentalReprojectionError, 4, 12, 3>(
            new FundamentalReprojectionError(norm_img1_points[i],
                                             norm_img2_points[i]));
    problem.AddResidualBlock(cost_function, NULL /* squared loss */,
                             right_projection.data(), world_points[i].data());
  }

  // Set up minimization options.
  ceres::Solver::Options options;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  *fundamental_matrix =
      img2_norm_mat.transpose() * CrossProductMatrix(right_projection.col(3)) *
      right_projection.block<3, 3>(0, 0) * img1_norm_mat;

  return true;
}

}  // namespace theia
