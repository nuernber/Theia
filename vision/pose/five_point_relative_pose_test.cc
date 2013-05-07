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

#include "vision/pose/five_point_relative_pose.h"

#include <Eigen/Core>
#include "gtest/gtest.h"
#include <vector>

#include "vision/models/essential_matrix.h"

namespace theia {
namespace {
const double kEps = 1e-12;
}

using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Vector3d;

TEST(FivePointRelativePose, Sanity) {
  // Ground truth essential matrix.
  Matrix3d essential_mat;
  essential_mat << 21.36410238362200, -6.45974435705903, -12.57571428668080,
      -7.57644257286238, -20.76739671331773, 28.55199769513539,
      23.52168039400320, -21.32072430358359, 1.00000000000000;

  // corresponding points
  double m1[5][3] = {{-0.06450996083506, 0.13271495297568, 1.0},
                     {0.12991134106070, 0.25636930852309, 1.0},
                     {-0.01417169260989, -0.00201819071777, 1.0},
                     {0.02077599827229, 0.14974189941659, 1.0},
                     {0.15970376410206, 0.17782283760921, 1.0}};
  double m2[5][3] = {{0.01991821260977, 0.13855920297370, 1.0},
                     {0.29792009989498, 0.21684093037950, 1.0},
                     {0.14221131487001, 0.03902000959363, 1.0},
                     {0.09012597508721, 0.11407993279726, 1.0},
                     {0.15373963107017, 0.02622710108650, 1.0}};

  std::vector<EssentialMatrix> essential_matrices =
      FivePointRelativePose(m1, m2);

  for (int i = 0; i < essential_matrices.size(); i++) {
    const Matrix3d& essential_matrix = essential_matrices[i].GetMatrix();
    Map<const Vector3d> u(m1[i]);
    Map<const Vector3d> u_prime(m2[i]);
    double epipolar_constraint = (u_prime.transpose())*essential_matrix*u;
    ASSERT_LT(fabs(epipolar_constraint), kEps);
  }
}
}  // namespace theia
