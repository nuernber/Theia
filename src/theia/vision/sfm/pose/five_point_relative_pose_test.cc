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
#include "theia/util/random.h"
#include "theia/vision/sfm/pose/five_point_relative_pose.h"
#include "theia/vision/sfm/pose/util.h"
#include "theia/test/benchmark.h"

namespace theia {
namespace {
using Eigen::AngleAxisd;
using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

double RotationAngularDistance(const Matrix3d& rot1, const Matrix3d& rot2) {
  const Matrix3d relative_rotation = rot1.transpose() * rot2;
  return acos((relative_rotation.trace() - 1.0) / 2.0);
}

// Tests that the three point pose works correctly by taking the passed
// points_3d, projecting them to get image one rays, transforming by
// (expected_rotation, expected_translation) to get image two rays and then
// verifying that the ThreePointEssentialMatrix function returns
// (test_rotation, test_translation_direction) among its solutions.
// Noise can be added to the image projections by setting projection_noise.
// The thresholds for rotation and translation similarity can be controlled
// by max_rotation_difference and max_difference_between_translation.
void TestFivePointResultWithNoise(const Vector3d points_3d[5],
                                  const double projection_noise_std_dev,
                                  const Matrix3d& expected_rotation,
                                  const Vector3d& expected_translation,
                                  const double max_rotation_difference,
                                  const double max_angle_between_translation) {
  InitRandomGenerator();

  // Calculates the image rays in both views.
  Vector3d view_one_rays[5];
  Vector3d view_two_rays[5];
  for (int i = 0; i < 5; ++i) {
    const Vector3d proj_3d =
        expected_rotation * points_3d[i] + expected_translation;
    view_one_rays[i] = points_3d[i].normalized();
    view_two_rays[i] = proj_3d.normalized();
  }

  if (projection_noise_std_dev) {
    // Adds noise to both of the rays.
    for (int i = 0; i < 5; ++i) {
      AddNoiseToProjection(projection_noise_std_dev,
                           &view_one_rays[i]);
      AddNoiseToProjection(projection_noise_std_dev,
                           &view_two_rays[i]);
    }
  }

  // Calculates the essential matrix, this may return multiple solutions.
  Matrix3d soln_rotations[10];
  Vector3d soln_translations[10];
  const int num_solutions = FivePointRelativePose(
      view_one_rays[0].data(), view_two_rays[0].data(),
      soln_rotations[0].data(), soln_translations[0].data());
  CHECK_GT(num_solutions, 0);

  // Among the returned solutions verify that at least one is close to the
  // expected translation and rotation.
  bool matched_transform = false;
  for (int n = 0; n < num_solutions; ++n) {
    Matrix3d essential_matrix = CrossProductMatrix(soln_translations[n]) *
                                soln_rotations[n];
    for (int i = 0; i < 5; ++i) {
      const double sampson_dist =
          SampsonDistance(essential_matrix, view_two_rays[i], view_one_rays[i]);
      EXPECT_NEAR(sampson_dist, 0.0, 1e-8)
          << "3d point = " << points_3d[i].transpose();
    }

    double rotation_difference =
        RotationAngularDistance(expected_rotation, soln_rotations[n]);

    bool matched_rotation = (rotation_difference < max_rotation_difference);

    // The translation is only known up to scale so this verifies that the
    // translations have matching directions.
    double translation_angle_difference =
        fabs(expected_translation.dot(soln_translations[n]) /
             (expected_translation.norm() * soln_translations[n].norm()));
    // Occasionally there is an odd floating point error that causes a nan in
    // the acos below, so we explicitly cap the angle values to be valid.
    translation_angle_difference =
        acos(std::min(1.0, std::max(0.0, translation_angle_difference)));

    bool matched_translation =
        (translation_angle_difference < max_angle_between_translation);

    if (matched_translation && matched_rotation) {
      matched_transform = true;
    }
  }
  EXPECT_TRUE(matched_transform);
}


void BasicTest() {
  // Ground truth essential matrix.
  const Vector3d points_3d[5] = { Vector3d(-1.0, 3.0, 3.0),
                                  Vector3d(1.0, -1.0, 2.0),
                                  Vector3d(3.0, 1.0, 2.5),
                                  Vector3d(-1.0, 1.0, 2.0),
                                  Vector3d(2.0, 1.0, 3.0) };
  const Matrix3d soln_rotation = Quaterniond(
      AngleAxisd(Radians(13.0), Vector3d(0.0, 0.0, 1.0))).toRotationMatrix();
  const Vector3d soln_translation(1.0, 1.0, 1.0);
  const double kNoise = 0.0 / 512.0;
  const double kMaxAllowedRotationDifference = 1e-5;
  const double kMaxAllowedAngleBetweenTranslations = 1e-5;

  TestFivePointResultWithNoise(points_3d,
                               kNoise,
                               soln_rotation,
                               soln_translation,
                               kMaxAllowedRotationDifference,
                               kMaxAllowedAngleBetweenTranslations);
}

TEST(FivePointRelativePose, Basic) {
  BasicTest();
}

BENCHMARK(FivePointRelativePose, BasicBenchmark, 100, 1000) {
  BasicTest();
}

TEST(FivePointRelativePose, NoiseTest) {
  // Ground truth essential matrix.
  const Vector3d points_3d[5] = {Vector3d(-1.0, 3.0, 3.0),
                                 Vector3d(1.0, -1.0, 2.0),
                                 Vector3d(3.0, 1.0, 2.5),
                                 Vector3d(-1.0, 1.0, 2.0),
                                 Vector3d(2.0, 1.0, 3.0)};
  const Matrix3d soln_rotation = Quaterniond(
      AngleAxisd(Radians(13.0), Vector3d(0.0, 0.0, 1.0))).toRotationMatrix();

  const Vector3d soln_translation(1.0, 1.0, 1.0);
  const double kNoise = 1.0 / 512.0;
  const double kMaxAllowedRotationDifference = Radians(1.0);
  const double kMaxAllowedAngleBetweenTranslations = Radians(1.0);

  TestFivePointResultWithNoise(points_3d,
                               kNoise,
                               soln_rotation,
                               soln_translation,
                               kMaxAllowedRotationDifference,
                               kMaxAllowedAngleBetweenTranslations);
}

TEST(FivePointRelativePose, ForwardMotion) {
  // Ground truth essential matrix.
  const Vector3d points_3d[5] = {Vector3d(-1.0, 3.0, 3.0),
                                 Vector3d(1.0, -1.0, 2.0),
                                 Vector3d(3.0, 1.0, 2.0),
                                 Vector3d(-1.0, 1.0, 2.0),
                                 Vector3d(2.0, 1.0, 3.0)};
  const Matrix3d soln_rotation = Quaterniond(
      AngleAxisd(Radians(13.0), Vector3d(0.0, 0.0, 1.0))).toRotationMatrix();

  const Vector3d soln_translation(0.0, 0.0, 1.0);
  const double kNoise = 1.0 / 512.0;
  const double kMaxAllowedRotationDifference = Radians(2.0);
  const double kMaxAllowedAngleBetweenTranslations = Radians(5.0);

  TestFivePointResultWithNoise(points_3d,
                               kNoise,
                               soln_rotation,
                               soln_translation,
                               kMaxAllowedRotationDifference,
                               kMaxAllowedAngleBetweenTranslations);
}

TEST(FivePointRelativePose, NoRotation) {
  // Ground truth essential matrix.
  const Vector3d points_3d[5] = {Vector3d(-1.0, 3.0, 3.0),
                                 Vector3d(1.0, -1.0, 2.0),
                                 Vector3d(3.0, 1.0, 2.0),
                                 Vector3d(-1.0, 1.0, 2.0),
                                 Vector3d(2.0, 1.0, 3.0)};
  const Matrix3d soln_rotation = Quaterniond(
      AngleAxisd(Radians(0.0), Vector3d(0.0, 0.0, 1.0))).toRotationMatrix();

  const Vector3d soln_translation(1.0, 1.0, 1.0);
  const double kNoise = 1.0 / 512.0;
  const double kMaxAllowedRotationDifference = Radians(1.0);
  const double kMaxAllowedAngleBetweenTranslations = Radians(1.0);

  TestFivePointResultWithNoise(points_3d,
                               kNoise,
                               soln_rotation,
                               soln_translation,
                               kMaxAllowedRotationDifference,
                               kMaxAllowedAngleBetweenTranslations);
}

}  // namespace
}  // namespace theia
