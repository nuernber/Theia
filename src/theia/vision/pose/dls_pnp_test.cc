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


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <algorithm>
#include <vector>
#include "gtest/gtest.h"

#include "theia/math/util.h"
#include "theia/vision/pose/dls_pnp.h"
#include "theia/vision/pose/util.h"
#include "theia/test/benchmark.h"

namespace theia {
namespace {
using Eigen::AngleAxisd;
using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

void TestDlsPnpWithNoise(const std::vector<Vector3d>& points_3d,
                         const double projection_noise_std_dev,
                         const Quaterniond& expected_rotation,
                         const Vector3d& expected_translation,
                         const double max_rotation_difference,
                         const double max_translation_difference) {
  const int num_points = points_3d.size();

  std::vector<Vector3d> camera_rays;
  std::vector<Vector3d> world_points;
  camera_rays.reserve(num_points);
  world_points.reserve(num_points);
  for (int i = 0; i < num_points; i++) {
    // Reproject 3D points into camera frame.
    camera_rays.push_back(points_3d[i].normalized());

    // Transform 3D points by solution transformation.
    Vector3d transformed_point =
        expected_rotation.inverse() * (points_3d[i] - expected_translation);
    world_points.push_back(transformed_point);
  }

  if (projection_noise_std_dev) {
    // Adds noise to both of the rays.
    for (int i = 0; i < num_points; i++) {
      AddNoiseToProjection(projection_noise_std_dev,
                           &camera_rays[i]);
    }
  }

  // Run DLS PnP.
  std::vector<Quaterniond> soln_rotation;
  std::vector<Vector3d> soln_translation;
  DlsPnp(camera_rays, world_points, &soln_rotation, &soln_translation);

  LOG(INFO) << "expected rot = " << expected_rotation.coeffs().transpose();
  LOG(INFO) << "expected trans = " << expected_translation.transpose();
  // Check solutions and verify at least one is close to the actual solution.
  const int num_solutions = soln_rotation.size();
  EXPECT_GT(num_solutions, 0);
  bool matched_transform = false;
  for (int i = 0; i < num_solutions; i++) {
    // Check that reprojection errors are small.
    for (int j = 0; j < num_points; j++) {
      const Vector3d reprojected_point =
          soln_rotation[i] * world_points[j] + soln_translation[i];
      LOG(INFO) << "reprojected_point = " << reprojected_point.normalized().transpose();
      LOG(INFO) << "camera ray = " << camera_rays[j].normalized().transpose();
      const double reprojection_error =
          ((camera_rays[j] / camera_rays[j].z()) -
           (reprojected_point / reprojected_point.z())).norm();
      EXPECT_LE(reprojection_error, 1e-4);
    }

    LOG(INFO) << "soln rot = " << soln_rotation[i].coeffs().transpose();
    LOG(INFO) << "soln trans = " << soln_translation[i].transpose();
    // Check that the solution is accurate.
    const double rotation_difference =
        expected_rotation.angularDistance(soln_rotation[i]);
    const bool matched_rotation =
        (rotation_difference < max_rotation_difference);
    const double translation_difference =
        (expected_translation - soln_translation[i]).squaredNorm();
    const bool matched_translation =
        (translation_difference < max_translation_difference);
    LOG(INFO) << "translation diff = " << translation_difference;
    LOG(INFO) << "rotation_diff = " << rotation_difference;
    if (matched_translation && matched_rotation) {
      matched_transform = true;
    }
  }
  EXPECT_TRUE(matched_transform);
}

void BasicTest() {
  // const std::vector<Vector3d> points_3d = { Vector3d(-1.0, 3.0, 3.0),
  //                                           Vector3d(1.0, -1.0, 2.0),
  //                                           Vector3d(-1.0, 1.0, 2.0),
  //                                           Vector3d(2.0, 1.0, 3.0) };
  // const Quaterniond soln_rotation = Quaterniond(
  //     AngleAxisd(Radians(13.0), Vector3d(0.0, 0.0, 1.0)));
  // const Vector3d soln_translation(1.0, 1.0, 1.0);
  const std::vector<Vector3d> points_3d = {
    Vector3d(-0.1949, 0.1959, 1).normalized(),
    Vector3d(0.0241, -0.0945, 1).normalized(),
    Vector3d(0.2723, -0.3873, 1).normalized(),
    Vector3d(0.1736, 0.0048, 1).normalized(),
    Vector3d(0.0028, 0.1574, 1).normalized(),
    Vector3d(0.3205, -0.0764, 1).normalized(),
    Vector3d(0.3701, -0.0313, 1).normalized(),
    Vector3d(-0.0292, 0.2992, 1).normalized(),
    Vector3d(-0.1665, -0.0415, 1).normalized(),
    Vector3d(0.3603, 0.2823, 1).normalized()
  };
  const Vector3d soln_translation(-0.7422, 0.5270, 2.1650);
  Matrix3d rot_mat;
  rot_mat << 0.9362, 0, -0.3514,
      0.0584, 0.9861, 0.1555,
      0.3465, -0.1661, 0.9232;
  const Quaterniond soln_rotation(rot_mat);
  LOG(INFO) << "soln_rotation = \n" << soln_rotation.toRotationMatrix();
  const double kNoise = 0.0;
  const double kMaxAllowedRotationDifference = 1e-5;
  const double kMaxAllowedTranslationDifference = 1e-8;

  TestDlsPnpWithNoise(points_3d,
                      kNoise,
                      soln_rotation,
                      soln_translation,
                      kMaxAllowedRotationDifference,
                      kMaxAllowedTranslationDifference);
}

TEST(DlsPnp, Basic) {
  BasicTest();
}

BENCHMARK(DlsPnp, BasicBenchmark, 100, 1000) {
  BasicTest();
}

TEST(DlsPnp, NoiseTest) {
}

TEST(DlsPnp, ManyPoints) {
}

TEST(DlsPnp, NoRotation) {
}

TEST(DlsPnp, NoTranslation) {
}

TEST(DlsPnp, OrthogonalRotation) {
}

}  // namespace
}  // namespace theia
