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

#include <math.h>
#include <Eigen/Core>
#include "gtest/gtest.h"
#include "vision/pose/perspective_three_point.h"

namespace vision {
namespace pose {
using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace {
double kEpsilon = 1e-9;

// Returns a random double between dMin and dMax
double RandDouble(double dMin, double dMax) {
  double d = static_cast<double>(rand()) / RAND_MAX;
  return dMin + d * (dMax - dMin);
}
}

TEST(PerspectiveThreePoint, Normalized) {
  // World coordinates of the 4 control points. Let them be random points in the
  // 2x2x2 centered around the origin.
  double world_points[4][3];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++)
      world_points[i][j] = RandDouble(-2, 2);

  // Find the camera projections of each of these points.
  // Make the camera at (0, 0, 6) looking straight down the axis.
  Matrix<double, 3, 4> transformation;
  transformation << 1, 0, 0, 0,
      0, -1, 0, 0,
      0, 0, -1, 8;

  double image_points[4][3];
  for (int i = 0; i < 4; i++) {
    Vector4d world_pt(world_points[i][0],
                      world_points[i][1],
                      world_points[i][2],
                      1.0);
    Vector3d proj_point = transformation*world_pt;
    image_points[i][0] = proj_point[0];
    image_points[i][1] = proj_point[1];
    image_points[i][2] = proj_point[2];
  }

  double rotation[4][3][3];
  double translation[4][3];
  int num_solutions = PoseThreePoints(image_points,
                                      world_points,
                                      rotation,
                                      translation);
  ASSERT_GT(num_solutions, 0);

  double best_rotation[3][3];
  double best_translation[3];
  bool solved = PoseFourPoints(image_points,
                               world_points,
                               best_rotation,
                               best_translation);

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      ASSERT_LT(std::abs(best_rotation[i][j] - transformation(i, j)), kEpsilon);
    }
    ASSERT_LT(std::abs(best_translation[i] - transformation(i, 3)), kEpsilon);
  }
}

}  // namespace pose
}  // namespace vision
