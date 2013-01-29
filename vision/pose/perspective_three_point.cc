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

#include "vision/pose/perspective_three_point.h"

#include <math.h>
#include <Eigen/Dense>
#include <algorithm>
#include <glog/logging.h>
#include "math/closed_form_polynomial_solver.h"

namespace vision {
namespace pose {
// Computes camera pose using the three point algorithm and returns all possible
// solutions (up to 4).
int PoseThreePoints(const double image_points[3][2],
                    const double world_points[3][3],
                    const double focal_length[2],
                    const double principle_point[2],
                    double rotation[][3][3],
                    double translation[][3]) {
  using Eigen::Matrix3d;
  using Eigen::Vector3d;

  // World points
  Vector3d p1(world_points[0][0], world_points[0][1], world_points[0][2]);
  Vector3d p2(world_points[1][0], world_points[1][1], world_points[1][2]);
  Vector3d p3(world_points[2][0], world_points[2][1], world_points[2][2]);

  // Make sure the points are not collinear.
  if ( (p2 - p1).cross(p3 - p1).squaredNorm() == 0)
    return 0;

  // Extract image points as feature vectors (unitary, normalized).
  Vector3d f1((image_points[0][0] - principle_point[0])/focal_length[0],
              (image_points[0][1] - principle_point[1])/focal_length[1],
              1.0);

  Vector3d f2((image_points[1][0] - principle_point[0])/focal_length[0],
              (image_points[1][1] - principle_point[1])/focal_length[1],
              1.0);
  Vector3d f3((image_points[2][0] - principle_point[0])/focal_length[0],
              (image_points[2][1] - principle_point[1])/focal_length[1],
              1.0);
  f1.normalize();
  f2.normalize();
  f3.normalize();

  // Create intermediate camera frame.
  Vector3d e1 = f1;
  Vector3d e3 = f1.cross(f2).normalized();
  Vector3d e2 = e3.cross(f1);

  Matrix3d t;
  t << e1, e2, e3;
  t.transposeInPlace();

  // Project vector into transformed camera frame.
  f3 = t*f3;

  // Reinforce that f3[2] > 0 for having theta in [0;pi]
  if (f3[2] > 0) {
    Vector3d f2((image_points[0][0] - principle_point[0])/focal_length[0],
                (image_points[0][1] - principle_point[1])/focal_length[1],
                1.0);
    f1.normalize();
    Vector3d f1((image_points[1][0] - principle_point[0])/focal_length[0],
                (image_points[1][1] - principle_point[1])/focal_length[1],
                1.0);
    f2.normalize();
    Vector3d f3((image_points[2][0] - principle_point[0])/focal_length[0],
                (image_points[2][1] - principle_point[1])/focal_length[1],
                1.0);
    f3.normalize();

    e1 = f1;
    e3 = f1.cross(f2).normalized();
    e2 = e3.cross(e1);

    t << e1, e2, e3;
    t.transposeInPlace();

    f3 = t*f3;

    p1 = Vector3d(world_points[1][0], world_points[1][1], world_points[1][2]);
    p2 = Vector3d(world_points[0][0], world_points[0][1], world_points[0][2]);
    p3 = Vector3d(world_points[2][0], world_points[2][1], world_points[2][2]);
  }

  // Create intermediate world frame.
  Vector3d n1 = (p2 - p1).normalized();
  Vector3d n3 = n1.cross(p3 - p1).normalized();
  Vector3d n2 = n3.cross(n1);
  Matrix3d n;
  n << n1, n2, n3;
  n.transposeInPlace();

  // Extract known parameters.
  p3 = n*(p3 - p1);
  double d_12 = (p2 - p1).norm();
  double f_1 = f3(0)/f3(2);
  double f_2 = f3(1)/f3(2);
  double p_1 = p3(0);
  double p_2 = p3(1);

  double cos_beta = f1.dot(f2);
  double b = 1.0/(1.0 - cos_beta*cos_beta) - 1.0;

  if (cos_beta < 0)
    b = -sqrt(b);
  else
    b = sqrt(b);

  // Definition of temporary variables for avoiding multiple computation
  double f_1_pw2 = f_1*f_1;
  double f_2_pw2 = f_2*f_2;
  double p_1_pw2 = p_1*p_1;
  double p_1_pw3 = p_1_pw2*p_1;
  double p_1_pw4 = p_1_pw3*p_1;
  double p_2_pw2 = p_2*p_2;
  double p_2_pw3 = p_2_pw2*p_2;
  double p_2_pw4 = p_2_pw3*p_2;
  double d_12_pw2 = d_12*d_12;
  double b_pw2 = b*b;

  // Computation of factors of 4th degree polynomial.
  double factors[5];
  factors[0] = -f_2_pw2*p_2_pw4 - p_2_pw4*f_1_pw2 - p_2_pw4;
  factors[1] = 2*p_2_pw3*d_12*b + 2*f_2_pw2*p_2_pw3*d_12*b -
      2*f_2*p_2_pw3*f_1*d_12;
  factors[2] = -f_2_pw2*p_2_pw2*p_1_pw2 - f_2_pw2*p_2_pw2*d_12_pw2*b_pw2 -
      f_2_pw2*p_2_pw2*d_12_pw2 + f_2_pw2*p_2_pw4 + p_2_pw4*f_1_pw2 +
      2*p_1*p_2_pw2*d_12 + 2*f_1*f_2*p_1*p_2_pw2*d_12*b -
      p_2_pw2*p_1_pw2*f_1_pw2 + 2*p_1*p_2_pw2*f_2_pw2*d_12 -
      p_2_pw2*d_12_pw2*b_pw2 - 2*p_1_pw2*p_2_pw2;

  factors[3] = 2*p_1_pw2*p_2*d_12*b + 2*f_2*p_2_pw3*f_1*d_12 -
      2*f_2_pw2*p_2_pw3*d_12*b - 2*p_1*p_2*d_12_pw2*b;

  factors[4] = -2*f_2*p_2_pw2*f_1*p_1*d_12*b + f_2_pw2*p_2_pw2*d_12_pw2 +
      2*p_1_pw3*d_12 - p_1_pw2*d_12_pw2 + f_2_pw2*p_2_pw2*p_1_pw2 - p_1_pw4 -
      2*f_2_pw2*p_2_pw2*p_1*d_12 + p_2_pw2*f_1_pw2*p_1_pw2 +
      f_2_pw2*p_2_pw2*d_12_pw2*b_pw2;

  // Computation of roots.
  double real_roots[4];
  int num_solutions = math::SolveQuarticReals(factors[0],
                                              factors[1],
                                              factors[2],
                                              factors[3],
                                              factors[4],
                                              real_roots);
  // Backsubstitution of each solution
  for (int i = 0; i < num_solutions; i++) {
    double cot_alpha = (-f_1*p_1/f_2-real_roots[i]*p_2 + d_12*b)/
        (-f_1*real_roots[i]*p_2/f_2 + p_1-d_12);

    double cos_theta = real_roots[i];
    double sin_theta = sqrt(1 - real_roots[i]*real_roots[i]);
    double sin_alpha = sqrt(1/(cot_alpha*cot_alpha + 1));
    double cos_alpha = sqrt(1 - sin_alpha*sin_alpha);

    if (cot_alpha < 0)
      cos_alpha = -cos_alpha;

    Vector3d c_nu(d_12*cos_alpha*(sin_alpha*b + cos_alpha),
                  cos_theta*d_12*sin_alpha*(sin_alpha*b + cos_alpha),
                  sin_theta*d_12*sin_alpha*(sin_alpha*b + cos_alpha));

    Vector3d c = p1 + n.transpose()*c_nu;


    Vector3d temp1(-cos_alpha, -sin_alpha*cos_theta, -sin_alpha*sin_theta);
    Vector3d temp2(sin_alpha, -cos_alpha*cos_theta, -cos_alpha*sin_theta);
    Vector3d temp3(0, -sin_theta, cos_theta);
    Matrix3d r;
    r << temp1, temp2, temp3;
    r.transposeInPlace();

    r = n.transpose()*r.transpose()*t;

    // Copy solution output variable.
    memcpy(rotation[i], r.data(), sizeof(r(0, 0))*9);
    memcpy(translation[i], c.data(), sizeof(c(0))*9);
  }
  return num_solutions;
}

// Computes pose using three point algorithm. The fourth correspondence is used
// to determine the best solution of the (up to 4) candidate solutions.
bool PoseFourPoints(const double image_points[4][3],
                    const double world_points[4][3],
                    double rotation[3][3],
                    double translation[3]) {
  using Eigen::Affine3d;
  using Eigen::Map;
  using Eigen::Vector3d;
  using Eigen::Matrix;

  double candidate_rotation[4][3][3];
  double candidate_translation[4][3];
  int num_valid_poses = PoseThreePoints(image_points,
                                        world_points,
                                        candidate_rotation,
                                        candidate_translation);
  // For each candidate pose, measure the reprojection error of the 4th point
  // and pick the best candidate pose.
  double min_reprojection_error = 1e9;
  int best_pose_index = 0;
  if (num_valid_poses == 0)
    return false;
  
  for (int i = 0; i < num_valid_poses; i++) {
    Map<const Matrix<double, 3, 3, Eigen::RowMajor> >
        rotation_mat(reinterpret_cast<const double*>(&candidate_rotation[i][0]));
    Map<const Vector3d>
        translation_mat(reinterpret_cast<const double*>(&candidate_translation[i][0]));
    Affine3d transformation;
    transformation.rotation() = rotation_mat;
    transformation.translation() = translation_mat;

    Map<const Vector3d> image_pt(reinterpret_cast<const double*>(&image_points[3][0]));
    Vector3d projected_point = transformation*image_pt;
    VLOG(0) << "old point = " << image_pt;
    VLOG(0) << "new point = " << projected_point;
    /*
    double reprojection_error =
        (projected_point[0] - image_points[3][0])*
        (projected_point[0] - image_points[3][0]) +
        (projected_point[1] - image_points[3][1])*
        (projected_point[1] - image_points[3][1]);
    if (reprojection_error < min_reprojection_error) {
      min_reprojection_error = reprojection_error;
      best_pose_index = i;
    }
    */
  }

  // Copy solution to output variables.
  memcpy(rotation,
         candidate_rotation[best_pose_index],
         sizeof(candidate_rotation[0][0][0])*9);
  memcpy(translation,
         candidate_translation[best_pose_index],
         sizeof(candidate_translation[0][0])*3);
  return true;
}
}  // pose
}  // vision
