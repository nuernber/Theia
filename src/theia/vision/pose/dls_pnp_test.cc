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
#include "theia/test/benchmark.h"
#include "theia/util/random.h"
#include "theia/util/util.h"
#include "theia/vision/pose/dls_pnp.h"
#include "theia/vision/pose/util.h"

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
                         const double max_reprojection_error,
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

  // Check solutions and verify at least one is close to the actual solution.
  const int num_solutions = soln_rotation.size();
  EXPECT_GT(num_solutions, 0);
  bool matched_transform = false;
  for (int i = 0; i < num_solutions; i++) {
    // Check that reprojection errors are small.
    for (int j = 0; j < num_points; j++) {
      const Vector3d reprojected_point =
          soln_rotation[i] * world_points[j] + soln_translation[i];
      const double reprojection_error =
          ((camera_rays[j] / camera_rays[j].z()) -
           (reprojected_point / reprojected_point.z())).norm();
      EXPECT_LE(reprojection_error, max_reprojection_error);
    }

    // Check that the solution is accurate.
    const double rotation_difference =
        expected_rotation.angularDistance(soln_rotation[i]);
    const bool matched_rotation =
        (rotation_difference < max_rotation_difference);
    const double translation_difference =
        (expected_translation - soln_translation[i]).squaredNorm();
    const bool matched_translation =
        (translation_difference < max_translation_difference);

    if (matched_translation && matched_rotation) {
      matched_transform = true;
    }
  }
  EXPECT_TRUE(matched_transform);
}

void BasicTest() {
  const std::vector<Vector3d> points_3d = { Vector3d(-1.0, 3.0, 3.0),
                                            Vector3d(1.0, -1.0, 2.0),
                                            Vector3d(-1.0, 1.0, 2.0),
                                            Vector3d(2.0, 1.0, 3.0) };
  const Quaterniond soln_rotation = Quaterniond(
      AngleAxisd(Radians(13.0), Vector3d(0.0, 0.0, 1.0)));
  const Vector3d soln_translation(1.0, 1.0, 1.0);
  const double kNoise = 0.0;
  const double kMaxReprojectionError = 1e-4;
  const double kMaxAllowedRotationDifference = 1e-5;
  const double kMaxAllowedTranslationDifference = 1e-8;

  TestDlsPnpWithNoise(points_3d,
                      kNoise,
                      soln_rotation,
                      soln_translation,
                      kMaxReprojectionError,
                      kMaxAllowedRotationDifference,
                      kMaxAllowedTranslationDifference);
}

TEST(DlsPnp, Basic) {
  BasicTest();
}

BENCHMARK(DlsPnp, BasicBenchmark, 100, 100) {
  BasicTest();
}

TEST(DlsPnp, NoiseTest) {
    const std::vector<Vector3d> points_3d = { Vector3d(-1.0, 3.0, 3.0),
                                              Vector3d(1.0, -1.0, 2.0),
                                              Vector3d(-1.0, 1.0, 2.0),
                                              Vector3d(2.0, 1.0, 3.0),
                                              Vector3d(-1.0, -3.0, 2.0),
                                              Vector3d(1.0, -2.0, 1.0),
                                              Vector3d(-1.0, 4.0, 2.0),
                                              Vector3d(-2.0, 2.0, 3.0)
    };
  const Quaterniond soln_rotation = Quaterniond(
      AngleAxisd(Radians(13.0), Vector3d(0.0, 0.0, 1.0)));
  const Vector3d soln_translation(1.0, 1.0, 1.0);
  const double kNoise = 1.0 / 512.0;
  const double kMaxReprojectionError = 5e-3;
  const double kMaxAllowedRotationDifference = 1e-3;
  const double kMaxAllowedTranslationDifference = 1e-3;

  TestDlsPnpWithNoise(points_3d,
                      kNoise,
                      soln_rotation,
                      soln_translation,
                      kMaxReprojectionError,
                      kMaxAllowedRotationDifference,
                      kMaxAllowedTranslationDifference);
}

TEST(DlsPnp, ManyPoints) {
// Sets some test rotations and translations.
  static const Vector3d kAxes[] = {
      Vector3d(0.0, 0.0, 1.0).normalized(),
      Vector3d(0.0, 1.0, 0.0).normalized(),
      Vector3d(1.0, 0.0, 0.0).normalized(),
      Vector3d(1.0, 0.0, 1.0).normalized(),
      Vector3d(0.0, 1.0, 1.0).normalized(),
      Vector3d(1.0, 1.0, 1.0).normalized(),
      Vector3d(0.0, 1.0, 1.0).normalized(),
      Vector3d(1.0, 1.0, 1.0).normalized()
  };

  static const double kRotationAngles[ARRAYSIZE(kAxes)] = {
      Radians(7.0),
      Radians(12.0),
      Radians(15.0),
      Radians(20.0),
      Radians(11.0),
      Radians(0.0),  // Tests no rotation.
      Radians(5.0),
      Radians(0.0)  // Tests no rotation and no translation.
  };

  static const Vector3d kTranslations[ARRAYSIZE(kAxes)] = {
      Vector3d(1.0, 1.0, 1.0),
      Vector3d(3.0, 2.0, 13.0),
      Vector3d(4.0, 5.0, 11.0),
      Vector3d(1.0, 2.0, 15.0),
      Vector3d(3.0, 1.5, 51.0),
      Vector3d(1.0, 7.0, 11.0),
      Vector3d(0.0, 0.0, 0.0),  // Tests no translation.
      Vector3d(0.0, 0.0, 0.0)  // Tests no translation and no rotation.
  };

  static const int num_points[3] = { 100, 500, 1000 };
  const double kNoise = 1.0 / 512.0;
  const double kMaxReprojectionError = 5e-3;
  const double kMaxAllowedRotationDifference = 5e-3;
  const double kMaxAllowedTranslationDifference = 1e-3;

  for (int i = 0; i < ARRAYSIZE(kAxes); i++) {
    const Quaterniond soln_rotation(AngleAxisd(kRotationAngles[i], kAxes[i]));
    for (int j = 0; j < ARRAYSIZE(num_points); j++) {
      std::vector<Vector3d> points_3d;
      points_3d.reserve(num_points[j]);
      for (int k = 0; k < num_points[j]; k++) {
        points_3d.push_back(Vector3d(RandDouble(-5.0, 5.0),
                                     RandDouble(-5.0, 5.0),
                                     RandDouble(2.0, 10.0)));
      }
      TestDlsPnpWithNoise(points_3d,
                          kNoise,
                          soln_rotation,
                          kTranslations[i],
                          kMaxReprojectionError,
                          kMaxAllowedRotationDifference,
                          kMaxAllowedTranslationDifference);
    }

  }
}

TEST(DlsPnp, NoRotation) {
    const std::vector<Vector3d> points_3d = { Vector3d(-1.0, 3.0, 3.0),
                                              Vector3d(1.0, -1.0, 2.0),
                                              Vector3d(-1.0, 1.0, 2.0),
                                              Vector3d(2.0, 1.0, 3.0),
                                              Vector3d(-1.0, -3.0, 2.0),
                                              Vector3d(1.0, -2.0, 1.0),
                                              Vector3d(-1.0, 4.0, 2.0),
                                              Vector3d(-2.0, 2.0, 3.0)
    };
  const Quaterniond soln_rotation = Quaterniond(
      AngleAxisd(Radians(0.0), Vector3d(0.0, 0.0, 1.0)));
  const Vector3d soln_translation(1.0, 1.0, 1.0);
  const double kNoise = 1.0 / 512.0;
  const double kMaxReprojectionError = 5e-3;
  const double kMaxAllowedRotationDifference = 2e-3;
  const double kMaxAllowedTranslationDifference = 1e-3;

  TestDlsPnpWithNoise(points_3d,
                      kNoise,
                      soln_rotation,
                      soln_translation,
                      kMaxReprojectionError,
                      kMaxAllowedRotationDifference,
                      kMaxAllowedTranslationDifference);
}

TEST(DlsPnp, NoTranslation) {
      const std::vector<Vector3d> points_3d = { Vector3d(-1.0, 3.0, 3.0),
                                              Vector3d(1.0, -1.0, 2.0),
                                              Vector3d(-1.0, 1.0, 2.0),
                                              Vector3d(2.0, 1.0, 3.0),
                                              Vector3d(-1.0, -3.0, 2.0),
                                              Vector3d(1.0, -2.0, 1.0),
                                              Vector3d(-1.0, 4.0, 2.0),
                                              Vector3d(-2.0, 2.0, 3.0)
    };
  const Quaterniond soln_rotation = Quaterniond(
      AngleAxisd(Radians(13.0), Vector3d(0.0, 0.0, 1.0)));
  const Vector3d soln_translation(0.0, 0.0, 0.0);
  const double kNoise = 1.0 / 512.0;
  const double kMaxReprojectionError = 1e-2;
  const double kMaxAllowedRotationDifference = 5e-3;
  const double kMaxAllowedTranslationDifference = 1e-3;

  TestDlsPnpWithNoise(points_3d,
                      kNoise,
                      soln_rotation,
                      soln_translation,
                      kMaxReprojectionError,
                      kMaxAllowedRotationDifference,
                      kMaxAllowedTranslationDifference);

}

TEST(DlsPnp, OrthogonalRotation) {
    const std::vector<Vector3d> points_3d = { Vector3d(-1.0, 3.0, 3.0),
                                              Vector3d(1.0, -1.0, 2.0),
                                              Vector3d(-1.0, 1.0, 2.0),
                                              Vector3d(2.0, 1.0, 3.0),
                                              Vector3d(-1.0, -3.0, 2.0),
                                              Vector3d(1.0, -2.0, 1.0),
                                              Vector3d(-1.0, 4.0, 2.0),
                                              Vector3d(-2.0, 2.0, 3.0)
    };
  const Quaterniond soln_rotation = Quaterniond(
      AngleAxisd(Radians(90.0), Vector3d(0.0, 0.0, 1.0)));
  const Vector3d soln_translation(1.0, 1.0, 1.0);
  const double kNoise = 1.0 / 512.0;
  const double kMaxReprojectionError = 5e-3;
  const double kMaxAllowedRotationDifference = 1e-3;
  const double kMaxAllowedTranslationDifference = 1e-3;

  TestDlsPnpWithNoise(points_3d,
                      kNoise,
                      soln_rotation,
                      soln_translation,
                      kMaxReprojectionError,
                      kMaxAllowedRotationDifference,
                      kMaxAllowedTranslationDifference);
}

}  // namespace
}  // namespace theia
