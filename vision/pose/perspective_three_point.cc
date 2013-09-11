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

#include <glog/logging.h>
#include <math.h>

#include <Eigen/Dense>
#include <algorithm>
#include "math/closed_form_polynomial_solver.h"
#include "vision/pose/util.h"

namespace theia {
using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace {

// Solves for cos(theta) that will describe the rotation of the plane from
// intermediate world frame to intermediate camera frame. The method returns the
// roots of a quartic (i.e. solutions to cos(alpha) ) and several factors that
// are needed for back-substitution.
int SolvePlaneRotation(const Vector3d normalized_image_points[3],
                       const Vector3d& intermediate_image_point,
                       const Vector3d& intermediate_world_point,
                       const double d_12,
                       double real_roots[4],
                       double cot_alphas[4],
                       double* b) {
  // Calculate these parameters ahead of time for reuse and
  // readability. Notation for these variables is consistent with the notation
  // from the paper.
  const double f_1 = intermediate_image_point[0] / intermediate_image_point[2];
  const double f_2 = intermediate_image_point[1] / intermediate_image_point[2];
  const double p_1 = intermediate_world_point[0];
  const double p_2 = intermediate_world_point[1];
  const double cos_beta =
      normalized_image_points[0].dot(normalized_image_points[1]);
  *b = 1.0 / (1.0 - cos_beta * cos_beta) - 1.0;

  if (cos_beta < 0) {
    *b = -sqrt(*b);
  } else {
    *b = sqrt(*b);
  }

  // Definition of temporary variables for readability in the coefficients
  // calculation.
  const double f_1_pw2 = f_1 * f_1;
  const double f_2_pw2 = f_2 * f_2;
  const double p_1_pw2 = p_1 * p_1;
  const double p_1_pw3 = p_1_pw2 * p_1;
  const double p_1_pw4 = p_1_pw3 * p_1;
  const double p_2_pw2 = p_2 * p_2;
  const double p_2_pw3 = p_2_pw2 * p_2;
  const double p_2_pw4 = p_2_pw3 * p_2;
  const double d_12_pw2 = d_12 * d_12;
  const double b_pw2 = (*b) * (*b);

  // Computation of coefficients of 4th degree polynomial.
  double coefficients[5];
  coefficients[0] = -f_2_pw2 * p_2_pw4 - p_2_pw4 * f_1_pw2 - p_2_pw4;
  coefficients[1] =
      2 * p_2_pw3 * d_12 * (*b) + 2 * f_2_pw2 * p_2_pw3 * d_12 * (*b) -
      2 * f_2 * p_2_pw3 * f_1 * d_12;
  coefficients[2] =
      -f_2_pw2 * p_2_pw2 * p_1_pw2 - f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2 -
      f_2_pw2 * p_2_pw2 * d_12_pw2 + f_2_pw2 * p_2_pw4 + p_2_pw4 * f_1_pw2 +
      2 * p_1 * p_2_pw2 * d_12 + 2 * f_1 * f_2 * p_1 * p_2_pw2 * d_12 * (*b) -
      p_2_pw2 * p_1_pw2 * f_1_pw2 + 2 * p_1 * p_2_pw2 * f_2_pw2 * d_12 -
      p_2_pw2 * d_12_pw2 * b_pw2 - 2 * p_1_pw2 * p_2_pw2;
  coefficients[3] =
      2 * p_1_pw2 * p_2 * d_12 * (*b) + 2 * f_2 * p_2_pw3 * f_1 * d_12 -
      2 * f_2_pw2 * p_2_pw3 * d_12 * (*b) - 2 * p_1 * p_2 * d_12_pw2 * (*b);
  coefficients[4] =
      -2 * f_2 * p_2_pw2 * f_1 * p_1 * d_12 * (*b) +
      f_2_pw2 * p_2_pw2 * d_12_pw2 + 2 * p_1_pw3 * d_12 - p_1_pw2 * d_12_pw2 +
      f_2_pw2 * p_2_pw2 * p_1_pw2 - p_1_pw4 -
      2 * f_2_pw2 * p_2_pw2 * p_1 * d_12 + p_2_pw2 * f_1_pw2 * p_1_pw2 +
      f_2_pw2 * p_2_pw2 * d_12_pw2 * b_pw2;

  // Computation of roots.
  const int num_solutions =
      SolveQuarticReals(coefficients[0], coefficients[1], coefficients[2],
                        coefficients[3], coefficients[4], real_roots);

  // Calculate cot(alpha) needed for back-substitution.
  for (int i = 0; i < num_solutions; i++) {
    cot_alphas[i] = (-f_1 * p_1 / f_2 - real_roots[i] * p_2 + d_12 * (*b)) /
                    (-f_1 * real_roots[i] * p_2 / f_2 + p_1 - d_12);
  }

  return num_solutions;
}

// Given the complete transformation between intermediate world and camera
// frames (parameterized by cos_theta and cot_alpha), back-substitute the
// solution and get an absolute camera pose.
void Backsubstitute(const Matrix3d& intermediate_world_frame,
                    const Matrix3d& intermediate_camera_frame,
                    const Vector3d& world_point_0,
                    const double cos_theta,
                    const double cot_alpha,
                    const double d_12,
                    const double b,
                    double solution_translation[3],
                    double solution_rotation[9]) {
  const double sin_theta = sqrt(1.0 - cos_theta * cos_theta);
  const double sin_alpha = sqrt(1.0 / (cot_alpha * cot_alpha + 1.0));
  double cos_alpha = sqrt(1.0 - sin_alpha * sin_alpha);

  if (cot_alpha < 0) {
    cos_alpha = -cos_alpha;
  }

  // Get the camera position in the intermediate world frame
  // coordinates. (Eq. 5 from the paper).
  const Vector3d c_nu(
      d_12 * cos_alpha * (sin_alpha * b + cos_alpha),
      cos_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha),
      sin_theta * d_12 * sin_alpha * (sin_alpha * b + cos_alpha));

  // Transform c_nu into world coordinates. Use a Map to put the solution
  // directly into the output.
  Map<Vector3d> translation(solution_translation);
  translation = world_point_0 + intermediate_world_frame.transpose() * c_nu;

  // Construct the transformation from the intermediate world frame to the
  // intermediate camera frame.
  Matrix3d intermediate_world_to_camera_rotation;
  intermediate_world_to_camera_rotation <<
      -cos_alpha, -sin_alpha * cos_theta, -sin_alpha * sin_theta,
      sin_alpha, -cos_alpha * cos_theta, -cos_alpha * sin_theta,
      0, -sin_theta, cos_theta;

  // Construct the rotation matrix.
  Map<Matrix3d> rotation(solution_rotation);
  rotation = (intermediate_world_frame.transpose() *
              intermediate_world_to_camera_rotation.transpose() *
              intermediate_camera_frame).transpose();

  // Adjust translation to account for rotation.
  translation = rotation * (-translation);
}

int PoseFromThreePoints(const double points_2d[2 * 3],
                        const double points_3d[3 * 3],
                        const double f[2],
                        const double c[2],
                        double solution_rotations[9 * 4],
                        double solution_translations[3 * 4]) {
  CHECK_GT(f[0], 0.0);
  CHECK_GT(f[1], 0.0);

  // Compute the 3D unit length vectors from camera center to each
  // image point. First undo the calibration matrix transformation:
  // x = (pt[0] - c_x) / f_x
  // y = (pt[1] - c_y) / f_y
  // z = 1.0
  // Then normalize those vectors to length 1.
  Vector3d normalized_image_points[3];
  // Store points_3d in world_points for ease of use. NOTE: we cannot use a
  // const ref or a Map because the world_points entries may be swapped later.
  Vector3d world_points[3];
  for (int i = 0; i < 3; ++i) {
    normalized_image_points[i][0] = (points_2d[2 * i + 0] - c[0]) / f[0];
    normalized_image_points[i][1] = (points_2d[2 * i + 1] - c[1]) / f[1];
    normalized_image_points[i][2] = 1.0;
    normalized_image_points[i].normalize();

    world_points[i] = Vector3d(points_3d[i * 3 + 0], points_3d[i * 3 + 1],
                               points_3d[i * 3 + 2]);
  }

  // If the points are collinear, there are no possible solutions.
  double kTolerance = 1e-6;
  const Vector3d world_1_0 = world_points[1] - world_points[0];
  const Vector3d world_2_0 = world_points[2] - world_points[0];
  if (world_1_0.cross(world_2_0).squaredNorm() < kTolerance) {
    LOG(INFO) << "The 3 world points are collinear! No solution for absolute "
                 "pose exits";
    return 0;
  }

  // Create intermediate camera frame such that the x axis is in the direction
  // of one of the normalized image points, and the origin is the same as the
  // absolute camera frame. This is a rotation defined as the transformation:
  // T = [tx, ty, tz] where tx = f0, tz = (f0 x f1) / ||f0 x f1||, and
  // ty = tx x tz and f0, f1, f2 are the normalized image points.
  Matrix3d intermediate_camera_frame;
  intermediate_camera_frame.row(0) = normalized_image_points[0];
  intermediate_camera_frame.row(2) =
      normalized_image_points[0].cross(normalized_image_points[1]).normalized();
  intermediate_camera_frame.row(1) =
      intermediate_camera_frame.row(2).cross(intermediate_camera_frame.row(0));

  // Project the third world point into the intermediate camera frame.
  Vector3d intermediate_image_point =
      intermediate_camera_frame * normalized_image_points[2];

  // Enforce that the intermediate_image_point is in front of the intermediate
  // camera frame. If the point is behind the camera frame, recalculate the
  // intermediate camera frame by swapping which feature we align the x axis to.
  if (intermediate_image_point[2] > 0) {
    std::swap(normalized_image_points[0], normalized_image_points[1]);

    intermediate_camera_frame.row(0) = normalized_image_points[0];
    intermediate_camera_frame.row(2) = normalized_image_points[0]
        .cross(normalized_image_points[1]).normalized();
    intermediate_camera_frame.row(1) = intermediate_camera_frame.row(2)
        .cross(intermediate_camera_frame.row(0));

    intermediate_image_point =
        intermediate_camera_frame * normalized_image_points[2];

    std::swap(world_points[0], world_points[1]);
  }

  // Create the intermediate world frame transformation where that has the
  // origin at world_points[0] and the x-axis in the direction of
  // world_points[1]. This is defined by the transformation: N = [nx, ny, nz]
  // where nx = (p1 - p0) / ||p1 - p0||
  // nz = nx x (p2 - p0) / || nx x (p2 -p0) || and ny = nz x nx
  // Where p0, p1, p2 are the world points.
  Matrix3d intermediate_world_frame;
  intermediate_world_frame.row(0) =
      (world_points[1] - world_points[0]).normalized();
  intermediate_world_frame.row(2) = intermediate_world_frame.row(0)
      .cross(world_points[2] - world_points[0]).normalized();
  intermediate_world_frame.row(1) =
      intermediate_world_frame.row(2).cross(intermediate_world_frame.row(0));

  // Transform world_point[2] to the intermediate world frame coordinates.
  Vector3d intermediate_world_point =
      intermediate_world_frame * (world_points[2] - world_points[0]);

  // Distance from world_points[1] to the intermediate world frame origin.
  double d_12 = (world_points[1] - world_points[0]).norm();

  // Solve for the cos(theta) that will give us the transformation from
  // intermediate world frame to intermediate camera frame. We also get the
  // cot(alpha) for each solution necessary for back-substitution.
  double real_roots[4];
  double cot_alphas[4];
  double b;
  const int num_solutions = SolvePlaneRotation(
      normalized_image_points, intermediate_image_point,
      intermediate_world_point, d_12, real_roots, cot_alphas, &b);

  // Backsubstitution of each solution
  for (int i = 0; i < num_solutions; i++) {
    Backsubstitute(intermediate_world_frame,
                   intermediate_camera_frame,
                   world_points[0],
                   real_roots[i],
                   cot_alphas[i],
                   d_12,
                   b,
                   solution_translations + i * 3,
                   solution_rotations + i * 9);
  }
  return num_solutions;
}

}  // namespace

int PoseFromThreeCalibrated(const double points_2d[2 * 3],
                            const double points_3d[3 * 3],
                            const double focal_length[2],
                            const double principal_point[2],
                            double solutions[12 * 4]) {
  double rotation_solutions[9 * 4];
  double translation_solutions[3 * 4];

  int n = PoseFromThreePoints(points_2d,
                              points_3d,
                              focal_length,
                              principal_point,
                              rotation_solutions,
                              translation_solutions);

  // Convert the output to projection matrices: P = K*[R(q)  t].
  for (int k = 0; k < n; ++k) {
    // Compose the projection matrix.
    ComposeProjectionMatrix(focal_length,
                            principal_point,
                            rotation_solutions + 9 * k,
                            translation_solutions + 3 * k,
                            solutions + 12 * k);
  }

  return n;
}

// Computes pose using three point algorithm. The fourth correspondence is used
// to determine the best solution of the (up to 4) candidate solutions.
bool PoseFourPointsCalibrated(const double image_points[2 * 4],
                              const double world_points[3 * 4],
                              const double focal_length[2],
                              const double principle_point[2],
                              double solution[12]) {
  double candidate_solutions[12 * 4];
  int num_valid_poses =
      PoseFromThreeCalibrated(image_points, world_points, focal_length,
                              principle_point, candidate_solutions);

  // For each candidate pose, measure the reprojection error of the 4th point
  // and pick the best candidate pose.
  double min_reprojection_error = 1e9;
  int best_pose_index = 0;
  if (num_valid_poses == 0) return false;

  const Vector3d pt_3d = Map<const Vector3d>(world_points + 3 * 3);
  const Vector3d image_pt(image_points[0 + 2 * 3], image_points[1 + 2 * 3],
                          1.0);
  for (int i = 0; i < num_valid_poses; i++) {
    // Project the 3d point into the image.
    Vector3d proj_3d =
        Map<const Matrix3d>(candidate_solutions + 12 * i) * pt_3d +
        Map<const Vector3d>(candidate_solutions + 12 * i + 9);
    // Normalize.
    proj_3d /= proj_3d[2];

    // Calculate the reprojection error.
    double reprojection_error = (proj_3d - image_pt).squaredNorm();
    if (min_reprojection_error > reprojection_error) {
      min_reprojection_error = reprojection_error;
      best_pose_index = i;
    }
  }

  // Copy solution to output variables.
  std::copy(candidate_solutions + best_pose_index * 12,
            candidate_solutions + best_pose_index * 12 + 12, solution);
  return true;
}

}  // namespace theia
