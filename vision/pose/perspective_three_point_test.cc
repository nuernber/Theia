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

#include "vision/pose/perspective_three_point.h"

#include <Eigen/Core>
#include <math.h>
#include "gtest/gtest.h"
#include "test/test_utils.h"
#include "util/random.h"

namespace theia {

using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Vector3d;

void PoseFromThreeCalibratedTest() {
  // Projection matrix.
  const double kP[] = { 0.846169, -0.0901237, -0.525239,
                       -0.130189, 0.920775, -0.367729,
                       0.516768, 0.379541, 0.767398,
                       -0.365021, -0.0120234, 4.66054};

  // Points in the 3D scene.
  const double kPoints3d[] = {-0.3001, -0.5840, -1.2271,
                              -1.4487, 0.6965, -0.3889,
                              -0.7815, 0.7642, -0.1257};

  // Points in the camera view.
  const double kPoints2d[] = {-0.2877, -0.2416,
                              -0.3868, 0.1258,
                              -0.2537, 0.1522};

  const double kFocalLength[] = {1.0, 1.0};
  const double kPrincipalPoint[] = {0.0, 0.0};

  double solutions[4*12];
  int num_solutions = PoseFromThreeCalibrated(kPoints2d,
                                              kPoints3d,
                                              kFocalLength,
                                              kPrincipalPoint,
                                              solutions);
  bool matched_transform = false;
  for (int i = 0; i < num_solutions; ++i) {
    if (test::ArraysEqualUpToScale(12, kP, solutions + i * 12, 1e-6)) {
      matched_transform = true;
      for (int n = 0; n < 3; ++n) {
        const Vector3d pt_3d =
            Map<const Vector3d> (kPoints3d + n * 3);
        const Vector3d proj_3d =
            Map<const Matrix3d>(kP) * pt_3d + Map<const Vector3d>(kP + 9);
        for (int d = 0; d < 2; ++d) {
          EXPECT_NEAR(kPoints2d[d + n * 2], proj_3d[d] / proj_3d[2], 1e-4);
        }
      }
    }
  }
  EXPECT_TRUE(matched_transform);
}

TEST(pose, PoseFromThreeCalibrated) {
  PoseFromThreeCalibratedTest();
}

// TODO(cmsweeney): Add benchmark.
// static void BM_PoseFromThreeCalibrated(int iters) {
//   for (int i = 0; i < iters; i++)
//     PoseFromThreeCalibratedTest();
// }

// BENCHMARK(BM_PoseFromThreeCalibrated);

}  // namespace theia
