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
// ARE DISCLAIMED. IN NO EVENT SHALL CHRIS SWEENEY BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#include "vision/models/essential_matrix.h"

#include <chrono>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <glog/logging.h>
#include <math.h>
#include <random>

#include "test/test_utils.h"

namespace theia {
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Map;
using Eigen::Vector3d;
using test::RandDouble;

namespace {
// Tolerance for accepting numbers are equal (for use in decomposition).
double kTolerance = 1e-9;
typedef Eigen::Matrix<double, 3, 3, Eigen::RowMajor> RowMatrix3d;

inline Matrix3d CrossProductMatrix(const Vector3d& vec) {
  Matrix3d cross_prod;
  cross_prod << 0, -vec(2), vec(1),
      vec(2), 0, -vec(0),
      -vec(1), vec(0), 0;
  return cross_prod;
}
}  // namespace

TEST(EssentialMatrix, Decompose) {
  // Construct a rotation for a small rotation.
  Matrix3d rotation;
  rotation = AngleAxisd(RandDouble(-0.2, 0.2), Vector3d::UnitX())
      *AngleAxisd(RandDouble(-0.2, 0.2), Vector3d::UnitY())
      *AngleAxisd(RandDouble(-0.2, 0.2), Vector3d::UnitZ());
  Vector3d trans = Vector3d::Random();
  Matrix3d translation = CrossProductMatrix(trans);

  // Before Eq. 9.11 on page 257 of Hartley and Zisserman.
  Matrix3d e = translation*rotation;
  e /= -e(2, 2);
  EssentialMatrix my_essential_mat(e);

  double my_rotation[4][3][3];
  double my_translation[4][3];
  my_essential_mat.Decompose(my_rotation, my_translation);

  for (int i = 0; i < 4; i++) {
    Map<const RowMatrix3d> rot(
        reinterpret_cast<double*>(my_rotation[i][0]));
    Map<const Vector3d> tran(
        reinterpret_cast<double*>(my_translation[i]));
    Matrix3d estimated_e = CrossProductMatrix(tran)*rot;
    // Fix estimated_e to be the same sign as e and scale be -1 at the bottom
    // right corner.
    estimated_e /= -estimated_e(2, 2);
    test::ExpectMatricesNear(estimated_e, e, kTolerance);
  }
}

TEST(EssentialMatrix, DecomposeWithIdealCorrespondence) {
  RowMatrix3d true_rotation;
  true_rotation = AngleAxisd(RandDouble(-0.2, 0.2), Vector3d::UnitX())
      *AngleAxisd(RandDouble(-0.2, 0.2), Vector3d::UnitY())
      *AngleAxisd(RandDouble(-0.2, 0.2), Vector3d::UnitZ());
  Vector3d true_trans(Vector3d::Random());
  Matrix3d trans = CrossProductMatrix(true_trans);

  // Generate random 3d points, making sure all of them are in front of the
  // camera. Then project them into the 2nd camera using the rotate/translate
  // transformation.
  double image1_pts[3];
  double image2_pts[3];
  double world_points[4] = {RandDouble(-3, 3),
                            RandDouble(-3, 3),
                            RandDouble(1, 5),
                            1.0};
  // Generate corresponding image point.
  Eigen::Matrix<double, 3, 4> transform;
  transform << true_rotation, true_trans;

  Map<const Eigen::Vector4d> world_pt(world_points);
  Map<Vector3d> img1_pt(image1_pts);
  img1_pt = world_pt.head(3);

  Map<Vector3d> img2_pt(image2_pts);
  img2_pt = transform*world_pt;

  // Generate Essential Matrix from the rotation/translation.
  EssentialMatrix my_essential_mat(trans*true_rotation);

  double rotation[3][3];
  double translation[3];

  my_essential_mat.DecomposeWithIdealCorrespondence(image1_pts,
                                                    image2_pts,
                                                    rotation,
                                                    translation);

  test::ExpectArraysNear(9,
                         reinterpret_cast<double*>(rotation),
                         true_rotation.data(),
                         kTolerance);
  test::ExpectArraysNear(3,
                         translation,
                         true_trans.normalized().data(),
                         kTolerance);
}
}  // namespace theia
