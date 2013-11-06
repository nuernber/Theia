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

#include "vision/transformation/align_point_clouds.h"

#include <Eigen/Dense>
#include <cmath>

#include "gtest/gtest.h"

namespace theia {
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::RowMajor;
using Eigen::Vector3d;

namespace {
double kEpsilon = 1e-6;
}  // namespace
TEST(AlignPointClouds, SimpleTransform) {
  // Make the model a 3x3 grid.
  Matrix<double, 9, 3, RowMajor> left_mat;
  left_mat << 0.4, -3.105, 2.147,
      1.293, 7.1982, -.068,
      -5.34, 0.708, -3.69,
      -.345, 1.987, 0.936,
      0.93, 1.45, 1.079,
      -3.15, -4.73, 2.49,
      2.401, -2.03, -1.87,
      3.192, -.573, 0.1,
      -2.53, 3.07, -5.19;

  // Reference the matrix as a 2d array.
  double (*left)[3] = (double (*)[3])left_mat.data();

  // Shift and rotate the points.
  Matrix3d rotation_mat;
  rotation_mat << 0, 0, 1.0,
      -1.0, 0, 0,
      0, -1.0, 0;
  Vector3d translation_vec(0, 2, 2);

  // Rotate the left points.
  Matrix<double, 3, 9> left_mat_transformed =
      rotation_mat*(left_mat.transpose());
  // Translate the left points.
  for (int i = 0; i < 9; i++)
    left_mat_transformed.col(i) += translation_vec;

  // Create a 2d accessor for the transformed points.
  double (*right)[3] = (double (*)[3])left_mat_transformed.data();

  double rotation[3][3];
  double translation[3];

  // Map the left points to the right points
  AlignPointClouds(left, right, 9, rotation, translation);

  // Ensure the calculated transformation is the same as the one we set.
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      ASSERT_LT(std::abs(rotation[i][j] - rotation_mat(i, j)), kEpsilon);
    }
    ASSERT_LT(std::abs(translation[i] - translation_vec(i)), kEpsilon);
  }
}
}  // namespace theia
