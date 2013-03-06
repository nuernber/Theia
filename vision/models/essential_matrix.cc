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

#include <Eigen/Dense>
#include <glog/logging.h>
#include <algorithm>

namespace vision {
namespace models {
using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace {
typedef Matrix<double, 3, 3, Eigen::RowMajor> RowMatrix3d;

// Performs the Cheirality Test to determine which of the candidate
// transformations results in the triangulated point in front of the two
// cameras. Appendix C of Nister paper.
int CheiralityTest(const Vector3d& q,
                   const Vector3d& q_prime,
                   const Vector3d& d,
                   const Matrix3d& essential_mat,
                   const double candidate_rotation[4][3][3],
                   const double candidate_translation[4][3]) {
  // Given the correspondance, find the one configuration where the
  // triangulation is in front of the camera.
  Vector3d eq = essential_mat*q;
  Vector3d c = q_prime.cross(Vector3d(eq(0), eq(1), 0.0));

  Map<const RowMatrix3d> rot(
      reinterpret_cast<const double*>(candidate_rotation[0]));
  Map<const Vector3d> trans(
      reinterpret_cast<const double*>(candidate_translation[0]));

  // Triangulate the 3d point.
  Matrix<double, 3, 4> p;
  p << rot, trans;
  Vector4d c_plane = p.transpose()*c;
  Vector4d triangulated_pt;
  triangulated_pt << d*c_plane(3),
      -d.dot(c_plane.head(3));
  // C1 and C2 test if the triangulated pt are in front of camera 1 and 2
  // respectively.
  double c1 = triangulated_pt(2)*triangulated_pt(3);
  double c2 = ((p*triangulated_pt)[2])*triangulated_pt(3);

  if (c1 < 0 && c2 < 0) {
    return 1;
  } else if (c1*c2 < 0) {
    Map<const RowMatrix3d> rot_temp(
        reinterpret_cast<const double*>(candidate_rotation[2]));
    Map<const Vector3d> trans_temp(
        reinterpret_cast<const double*>(candidate_translation[2]));

    // Triangulate the 3d point for the reflection configuration.
    p << rot_temp, trans_temp;
    c_plane = p.transpose()*c;
    triangulated_pt << d*c_plane(3),
        -d.dot(c_plane.head(3));
    c1 = triangulated_pt(2)*triangulated_pt(3);
    c2 = ((p*triangulated_pt)[2])*triangulated_pt(3);
    if (c1 > 0 && c2 > 0)
      return 2;
    else
      return 3;
  }

  return 0;
}

}  // namespace

EssentialMatrix::EssentialMatrix(const double data[3][3]) {
  essential_mat_ << data[0][0], data[0][1], data[0][2],
      data[1][0], data[1][1], data[1][2],
      data[2][0], data[2][1], data[2][2];
}

void EssentialMatrix::Decompose(double rotation[4][3][3],
                                double translation[4][3]) const {
  Matrix3d d;
  d << 0, 1, 0,
      -1, 0, 0,
      0, 0, 1;

  const Vector3d& ea = essential_mat_.row(0);
  const Vector3d& eb = essential_mat_.row(1);
  const Vector3d& ec = essential_mat_.row(2);

  // Generate cross products.
  Matrix3d cross_products;
  cross_products << ea.cross(eb), ea.cross(ec), eb.cross(ec);

  // Choose the cross product with the largest norm (for numerical accuracy).
  const Vector3d cf_scales(cross_products.col(0).squaredNorm(),
                           cross_products.col(1).squaredNorm(),
                           cross_products.col(2).squaredNorm());
  int max_index;
  double max_cf = cf_scales.maxCoeff(&max_index);

  // For index 0, 1, we want ea and for index 2 we want eb.
  int max_e_index = max_index/2;

  // Construct v of the SVD.
  Matrix3d v = Matrix3d::Zero();
  v.col(2) = cross_products.col(max_index).normalized();
  v.col(0) = essential_mat_.row(max_e_index).normalized();
  v.col(1) = v.col(2).cross(v.col(0));

  // Construct U of the SVD.
  Matrix3d u = Matrix3d::Zero();
  u.col(0) = (essential_mat_*v.col(0)).normalized();
  u.col(1) = (essential_mat_*v.col(1)).normalized();
  u.col(2) = u.col(0).cross(u.col(1));

  // Possible rotation configurations.
  const RowMatrix3d ra = u*d*v.transpose();
  const RowMatrix3d rb = u*d.transpose()*v.transpose();

  // Scale t to be proper magnitude. Scale factor is derived from the fact that
  // U*diag*V^t = E. We simply choose to scale it such that the last terms will
  // be equal.
  const Vector3d t = u.col(2).normalized();
  const Vector3d t_neg = -t;

  // Copy the 4 possible decompositions into the output arrays.
  std::copy(ra.data(), ra.data() + ra.size(),
            reinterpret_cast<double*>(rotation[0]));
  std::copy(t.data(), t.data() + t.size(), translation[0]);
  std::copy(ra.data(), ra.data() + ra.size(),
            reinterpret_cast<double*>(rotation[1]));
  std::copy(t_neg.data(), t_neg.data() + t_neg.size(), translation[1]);
  std::copy(rb.data(), rb.data() + rb.size(),
            reinterpret_cast<double*>(rotation[2]));
  std::copy(t.data(), t.data() + t.size(), translation[2]);
  std::copy(rb.data(), rb.data() + rb.size(),
            reinterpret_cast<double*>(rotation[3]));
  std::copy(t_neg.data(), t_neg.data() + t_neg.size(), translation[3]);
}

void EssentialMatrix::DecomposeWithIdealCorrespondence(
    const double image_point1[3],
    const double image_point2[3],
    double rotation[3][3],
    double translation[3]) const {
  // Map the image points to vectors.
  Map<const Vector3d> q(image_point1);
  Map<const Vector3d> q_prime(image_point2);

  double candidate_rotation[4][3][3];
  double candidate_translation[4][3];
  Decompose(candidate_rotation, candidate_translation);

  int best_index = CheiralityTest(q,
                                  q_prime,
                                  q,
                                  essential_mat_,
                                  candidate_rotation,
                                  candidate_translation);
  Map<const RowMatrix3d> rot_best(
      reinterpret_cast<const double*>(candidate_rotation[best_index]));
  Map<const Vector3d> trans_best(
      reinterpret_cast<const double*>(candidate_translation[best_index]));

  std::copy(rot_best.data(), rot_best.data() + rot_best.size(),
            reinterpret_cast<double*>(rotation));
  std::copy(trans_best.data(), trans_best.data() + trans_best.size(),
            reinterpret_cast<double*>(translation));

  return;
}

void EssentialMatrix::DecomposeWithCorrespondence(
    int num_correspondences,
    const double image_point1[][3],
    const double image_point2[][3],
    double rotation[3][3],
    double translation[3]) const {
  // Map the image points to vectors.

  double candidate_rotation[4][3][3];
  double candidate_translation[4][3];
  Decompose(candidate_rotation, candidate_translation);

  for (int i = 0; i < 4; i++) {
    VLOG(0) << "rotation\n";
    for (int j = 0; j < 3; j++)
      VLOG(0) << candidate_rotation[i][j][0] << " "
              << candidate_rotation[i][j][1] << " "
              << candidate_rotation[i][j][2];
    VLOG(0) << "translation\n" << candidate_translation[i][0] << " "
            << candidate_translation[i][1] << " "
            << candidate_translation[i][2];
  }

  // Given the correspondances, test each point to see which camera
  // configuration it will be in front of both cameras for.
  int support[4] = {0, 0, 0, 0};
  for (int i = 0; i < num_correspondences; i++) {
    Map<const Vector3d> q(image_point1[i]);
    Map<const Vector3d> q_prime(image_point2[i]);

    Vector3d a = essential_mat_.transpose()*q_prime;
    Vector3d b = q.cross(Vector3d(a(0), a(1), 0.0));
    Vector3d d = a.cross(b);
    int best_in = CheiralityTest(q,
                                 q_prime,
                                 d,
                                 essential_mat_,
                                 candidate_rotation,
                                 candidate_translation);
    support[best_in]++;
  }

  VLOG(0) << "support = " << support[0] << ", "
          << support[1] << ", "
          << support[2] << ", "
          << support[3];

  int best_index = 0;
  for (int i = 1; i < 4; i++) {
    best_index = support[i] > support[best_index] ? i : best_index;
  }

  Map<const RowMatrix3d> rot_best(
      reinterpret_cast<const double*>(candidate_rotation[best_index]));
  Map<const Vector3d> trans_best(
      reinterpret_cast<const double*>(candidate_translation[best_index]));

  std::copy(rot_best.data(), rot_best.data() + rot_best.size(),
            reinterpret_cast<double*>(rotation));
  std::copy(trans_best.data(), trans_best.data() + trans_best.size(),
            reinterpret_cast<double*>(translation));
  return;
}

std::ostream& operator <<(std::ostream& os,
                          const EssentialMatrix& mat) {
  os << mat.essential_mat_;
  return os;
}

}  // namespace models
}  // namespace vision
