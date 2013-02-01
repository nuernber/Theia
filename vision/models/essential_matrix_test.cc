// Copyright (C) 2013  Chris Sweeney <cmsweeney@cs.ucsb.edu>
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
//     * Neither the name of the University of California, Santa Barbara nor the
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

#include "vision/models/essential_matrix.h"

#include "gtest/gtest.h"
#include <glog/logging.h>
#include <Eigen/Dense>
#include "test/test_utils.h"

namespace vision {
namespace models {
using Eigen::Matrix3d;
using Eigen::Vector3d;
using test::RandDouble;

namespace {
Matrix3d CrossProductMatrix(const Vector3d& vec) {
  Matrix3d cross_prod;
  cross_prod << 0, -vec(2), vec(1),
      vec(2), 0, -vec(0),
      -vec(1), vec(0), 0;
  return cross_prod;
}
}  // namespace

TEST(EssentialMat, Decompose) {
  using Eigen::AngleAxisd;

  // Tolerance for accepting the decomposition.
  double kTolerance = 1e-12;
  
  Matrix3d rotation;
  rotation = AngleAxisd(RandDouble(-0.2, 0.2), Vector3d::UnitX())
      *AngleAxisd(RandDouble(-0.2, 0.2), Vector3d::UnitY())
      *AngleAxisd(RandDouble(-0.2, 0.2), Vector3d::UnitZ());
  Vector3d trans = Vector3d::Random();
  Matrix3d translation = CrossProductMatrix(trans);

  Matrix3d e = translation*rotation;
  EssentialMatrix my_essential_mat(e);

  double my_rotation[4][3][3];
  double my_translation[4][3];
  my_essential_mat.Decompose(my_rotation, my_translation);

  for (int i = 0; i < 4; i++) {
    Eigen::Map<const Matrix3d> rot(
        reinterpret_cast<double*>(my_rotation[i][0]));
    Eigen::Map<const Vector3d> tran(
        reinterpret_cast<double*>(my_translation[i]));
    Matrix3d estimated_e = CrossProductMatrix(tran)*rot;
    // Fix estimated_e to be the same sign as e.
    if (estimated_e(0, 0)*e(0, 0) < 0.0)
      estimated_e *= -1.0;
    
    test::ExpectMatricesNear(estimated_e, e, kTolerance);
  }
}

}  // namespace models
}  // namespace vision
