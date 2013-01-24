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

#include "vision/pose/five_point_relative_pose.h"

#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include "math/matrix/gauss_jordan.h"
#include "math/polynomial.h"
#include "vision/models/essential_matrix.h"

namespace vision {
namespace pose {
using Eigen::Matrix;
using Eigen::RowVector4d;
using vision::models::EssentialMatrix;

namespace {
// Multiplies two polynomials over the same variable.
template<int n1, int n2>
Matrix<double, 1, n1 + n2 - 1> MultiplyPoly(const Matrix<double, 1, n1>& a,
                                            const Matrix<double, 1, n2>& b) {
  Matrix<double, 1, n1 + n2 - 1> poly =
      Matrix<double, 1, n1 + n2 -1>::Zero();
  for (int i = 0; i < a.cols(); i++)
    for (int j = 0; j < b.cols(); j++)
      poly[i + j] += a[i]*b[j];

  return poly;
}

// Evaluates a given polynomial at the value x.
template<int n> double EvaluatePoly(const Matrix<double, 1, n>& poly,
                                    double x) {
  double val = 0;
  for (int i = poly.cols() - 1; i > 0; i--) {
    val += poly[i];
    val *= x;
  }
  val += poly[0];
  return val;
}

// Multiply two degree one polynomials of variables x, y, z.
// E.g. p1 = a[0]x + a[1]y + a[2]z + a[3]
// x^2 y^2 z^2 xy xz yz x y z 1
Matrix<double, 1, 10> MultiplyDegOnePoly(const RowVector4d& a,
                                         const RowVector4d& b) {
  Matrix<double, 1, 10> output;
  output(0) = a(0)*b(0);
  output(1) = a(1)*b(1);
  output(2) = a(2)*b(2);
  output(3) = a(0)*b(1) + a(1)*b(0);
  output(4) = a(0)*b(2) + a(2)*b(0);
  output(5) = a(1)*b(2) + a(2)*b(1);
  output(6) = a(0)*b(3) + a(3)*b(0);
  output(7) = a(1)*b(3) + a(3)*b(1);
  output(8) = a(2)*b(3) + a(3)*b(2);
  output(9) = a(3)*b(3);
  return output;
}

// Multiply a 2 deg poly (in x, y, z) and a one deg poly.
// x^3 y^3 x^2y xy^2 x^2z x^2 y^2z y^2 xyz xy | z^2x zx x z^2y zy y z^3 z^2 z 1
// NOTE: after the | all are variables along z.
Matrix<double, 1, 20> MultiplyDegTwoDegOnePoly(const Matrix<double, 1, 10>& a,
                                               const RowVector4d& b) {
  Matrix<double, 1, 20> output;
  output(0) = a(0)*b(0);
  output(1) = a(1)*b(1);
  output(2) = a(0)*b(1) + a(3)*b(0);
  output(3) = a(1)*b(0) + a(3)*b(1);
  output(4) = a(0)*b(2) + a(4)*b(0);
  output(5) = a(0)*b(3) + a(6)*b(0);
  output(6) = a(1)*b(2) + a(5)*b(1);
  output(7) = a(1)*b(3) + a(7)*b(1);
  output(8) = a(3)*b(2) + a(4)*b(1) + a(5)*b(0);
  output(9) = a(3)*b(3) + a(6)*b(1) + a(7)*b(0);
  output(10) = a(2)*b(0) + a(4)*b(2);
  output(11) = a(4)*b(3) + a(8)*b(0) + a(6)*b(2);
  output(12) = a(6)*b(3) + a(9)*b(0);
  output(13) = a(2)*b(1) + a(5)*b(2);
  output(14) = a(5)*b(3) + a(8)*b(1) + a(7)*b(2);
  output(15) = a(7)*b(3) + a(9)*b(1);
  output(16) = a(2)*b(2);
  output(17) = a(2)*b(3) + a(8)*b(2);
  output(18) = a(8)*b(3) + a(9)*b(2);
  output(19) = a(9)*b(3);
  return output;
}

// Shorthand for multiplying the Essential matrix with its transpose according
// to Eq. 20 in Nister paper.
Matrix<double, 1, 10> EETranspose(
    const Eigen::Matrix<double, 9, 4>& null_matrix,
    int i,
    int j) {
  return  MultiplyDegOnePoly(null_matrix.row(3*i), null_matrix.row(3*j)) +
      MultiplyDegOnePoly(null_matrix.row(3*i + 1), null_matrix.row(3*j + 1)) +
      MultiplyDegOnePoly(null_matrix.row(3*i + 2), null_matrix.row(3*j + 2));
}

// Builds the 10x20 constraint matrix according to Section 3.2.2 of Nister
// paper. Constraints are built based on the singularity of the Essential
// matrix, and the trace equation (Eq. 6). This builds the 10x20 matrix such
// that the columns correspond to: x^3, yx^2, y^2x, y^3, zx^2, zyx, zy^2, z^2x,
// z^2y, z^3, x^2, yx, y^2, zx, zy, z^2, x, y, z, 1.
Matrix<double, 10, 20> BuildConstraintMatrix(
    const Eigen::Matrix<double, 9, 4>& null_space) {
  Matrix<double, 10, 20> constraint_matrix;
  // Singularity constraint.
  constraint_matrix.row(0) =
      MultiplyDegTwoDegOnePoly(
          MultiplyDegOnePoly(null_space.row(1), null_space.row(5)) -
          MultiplyDegOnePoly(null_space.row(2), null_space.row(4)),
          null_space.row(6)) +
      MultiplyDegTwoDegOnePoly(
          MultiplyDegOnePoly(null_space.row(2), null_space.row(3)) -
          MultiplyDegOnePoly(null_space.row(0), null_space.row(5)),
          null_space.row(7)) +
      MultiplyDegTwoDegOnePoly(
          MultiplyDegOnePoly(null_space.row(0), null_space.row(4)) -
          MultiplyDegOnePoly(null_space.row(1), null_space.row(3)),
          null_space.row(8));

  // Trace Constraint. Only need to compute the upper triangular part of the
  // symmetric polynomial matrix
  Matrix<double, 1, 10> symmetric_poly[3][3];
  symmetric_poly[0][0] = EETranspose(null_space, 0, 0);
  symmetric_poly[1][1] = EETranspose(null_space, 1, 1);
  symmetric_poly[2][2] = EETranspose(null_space, 2, 2);

  Matrix<double, 1, 10> half_trace = 0.5*(symmetric_poly[0][0] +
                                          symmetric_poly[1][1] +
                                          symmetric_poly[2][2]);

  symmetric_poly[0][0] -= half_trace;
  symmetric_poly[1][1] -= half_trace;
  symmetric_poly[2][2] -= half_trace;
  symmetric_poly[0][1] = EETranspose(null_space, 0, 1);
  symmetric_poly[0][2] = EETranspose(null_space, 0, 2);
  symmetric_poly[1][0] = symmetric_poly[0][1];
  symmetric_poly[1][2] = EETranspose(null_space, 1, 2);
  symmetric_poly[2][0] = symmetric_poly[0][2];
  symmetric_poly[2][1] = symmetric_poly[1][2];

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      constraint_matrix.row(3*i + j + 1) =
          MultiplyDegTwoDegOnePoly(symmetric_poly[i][0],
                                   null_space.row(j)) +
          MultiplyDegTwoDegOnePoly(symmetric_poly[i][1],
                                   null_space.row(3 + j)) +
          MultiplyDegTwoDegOnePoly(symmetric_poly[i][2],
                                   null_space.row(6 + j));
    }
  }
  return constraint_matrix;
}

// Efficient nullspace extraction based on the idea of QR decomposition on the 5
// correspondences of the epipolar constraint. Instead of QR, we use
// Gauss-Jordan for the same effect.
Matrix<double, 9, 4> EfficientNullspaceExtraction(
    const Matrix<double, 5, 9>& constraint) {
  Matrix<double, 5, 9> constraint_copy(constraint);
  math::matrix::GaussJordan(&constraint_copy);
  Matrix<double, 4, 9> null_space;
  null_space << constraint_copy.rightCols(4).transpose(),
      -Eigen::Matrix4d::Identity();
  return null_space.transpose();
}

}  // namespace

// Implementation of Nister from "An Efficient Solution to the Five-Point
// Relative Pose Problem"
std::vector<EssentialMatrix> FivePointRelativePose(
    const double image1_points[5][2],
    const double image2_points[5][2]) {
  using Eigen::Vector3d;
  using Eigen::RowVector3d;

  // Step 1. Create the 5x9 matrix containing epipolar constraints.
  //   Essential matrix is a linear combination of the 4 vectors spanning the
  //   null space of this matrix (found by SVD).
  Matrix<double, 5, 9> epipolar_constraint;
  for (int i = 0; i < 5; i++) {
    // Make image points homogeneous.
    Vector3d tmp_img1(image1_points[i][0], image1_points[i][1], 1.0);
    Vector3d tmp_img2(image2_points[i][0], image2_points[i][1], 1.0);
    // Fill matrix with the epipolar constraint from q'_t*E*q = 0. Where q is
    // from the first image, and q' is from the second. Eq. 8 in the Nister
    // paper.
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++)
        epipolar_constraint(i, 3*j + k) = tmp_img1(k)*tmp_img2(j);
  }

  // Solve for right null space of the 5x9 matrix. NOTE: We use a
  // super-efficient method that is a variation of the QR decomposition
  // described in the Nister paper.  by roughly 5x.
  Eigen::Matrix<double, 9, 4> null_space =
      EfficientNullspaceExtraction(epipolar_constraint);

  // Step 2. Expansion of the epipolar constraints Eq. 5 and 6 from Nister
  // paper.
  Matrix<double, 10, 20> constraint_matrix = BuildConstraintMatrix(null_space);

  // Step 3. Gauss-Jordan Elimination with partial pivoting on constraint
  // matrix.
  math::matrix::GaussJordan(&constraint_matrix);

  // Step 4. Expand determinant polynomial of 3x3 polynomial B.
  // Create matrix B. Horribly ugly, but not sure if there's a better way to do
  // it!
  RowVector4d b11(constraint_matrix(4, 12),
                  constraint_matrix(4, 11) - constraint_matrix(5, 12),
                  constraint_matrix(4, 10) - constraint_matrix(5, 11),
                  -constraint_matrix(5, 10));
  RowVector4d b12(constraint_matrix(4, 15),
                  constraint_matrix(4, 14) - constraint_matrix(5, 15),
                  constraint_matrix(4, 13) - constraint_matrix(5, 14),
                  -constraint_matrix(5, 13));
  Matrix<double, 1, 5> b13;
  b13 << constraint_matrix(4, 19),
      constraint_matrix(4, 18) - constraint_matrix(5, 19),
      constraint_matrix(4, 17) - constraint_matrix(5, 18),
      constraint_matrix(4, 16) - constraint_matrix(5, 17),
      -constraint_matrix(5, 16);
  RowVector4d b21(constraint_matrix(6, 12),
                  constraint_matrix(6, 11) - constraint_matrix(7, 12),
                  constraint_matrix(6, 10) - constraint_matrix(7, 11),
                  -constraint_matrix(7, 10));
  RowVector4d b22(constraint_matrix(6, 15),
                  constraint_matrix(6, 14) - constraint_matrix(7, 15),
                  constraint_matrix(6, 13) - constraint_matrix(7, 14),
                  -constraint_matrix(7, 13));
  Matrix<double, 1, 5> b23;
  b23 << constraint_matrix(6, 19),
      constraint_matrix(6, 18) - constraint_matrix(7, 19),
      constraint_matrix(6, 17) - constraint_matrix(7, 18),
      constraint_matrix(6, 16) - constraint_matrix(7, 17),
      -constraint_matrix(7, 16);
  RowVector4d b31(constraint_matrix(8, 12),
                  constraint_matrix(8, 11) - constraint_matrix(9, 12),
                  constraint_matrix(8, 10) - constraint_matrix(9, 11),
                  -constraint_matrix(9, 10));
  RowVector4d b32(constraint_matrix(8, 15),
                  constraint_matrix(8, 14) - constraint_matrix(9, 15),
                  constraint_matrix(8, 13) - constraint_matrix(9, 14),
                  -constraint_matrix(9, 13));
  Matrix<double, 1, 5> b33;
  b33 << constraint_matrix(8, 19),
      constraint_matrix(8, 18) - constraint_matrix(9, 19),
      constraint_matrix(8, 17) - constraint_matrix(9, 18),
      constraint_matrix(8, 16) - constraint_matrix(9, 17),
      -constraint_matrix(9, 16);

  // Eq. 24.
  Matrix<double, 1, 8> p1 = MultiplyPoly(b12, b23) - MultiplyPoly(b13, b22);
  // Eq. 25.
  Matrix<double, 1, 8> p2 = MultiplyPoly(b13, b21) - MultiplyPoly(b11, b23);
  // Eq. 26.
  Matrix<double, 1, 7> p3 = MultiplyPoly(b11, b22) - MultiplyPoly(b12, b21);

  // Eq. 27. Form determinant of B as a 10th degree polynomial.
  Matrix<double, 1, 11> n = MultiplyPoly(p1, b31) + MultiplyPoly(p2, b32) +
      MultiplyPoly(p3, b33);

  // Step 5. Extract real roots of the 10th degree polynomial.
  std::vector<EssentialMatrix> essential_matrices;
  math::Polynomial determinant_poly(10, n.data());
  std::vector<double> roots = determinant_poly.RealRoots();
  for (double z : roots) {
    double x = EvaluatePoly(p1, z)/EvaluatePoly(p3, z);
    double y = EvaluatePoly(p2, z)/EvaluatePoly(p3, z);
    Matrix<double, 9, 1> temp_sum = x*null_space.col(0) +
        y*null_space.col(1) + z*null_space.col(2) + null_space.col(3);
    // Need to do it like this because temp_sum is a row vector and recasting
    // it as a 3x3 will load it column-major.
    Eigen::Matrix3d candidate_essential_mat;
    candidate_essential_mat << temp_sum.head(3).transpose(),
        temp_sum.segment(3, 3).transpose(),
        temp_sum.tail(3).transpose();
    essential_matrices.push_back(candidate_essential_mat);
  }

  // Step 5 (alternative). Extract roots of n (the 10th degree polynomial).
  // You can solve the 10 deg polynomial as a sturn sequence, or as an
  // eigen-decomposition of a companion matrix. We choose the latter because of
  // the efficiency and convenience of the Eigen library.
  /*
    Matrix<double, 10, 10> companion = Matrix<double, 10, 10>::Zero();
    // Scale n so that the 10th deg coefficient = 1.
    n /= n(10);
    // Construct companion matrix.
    companion.row(0) = -1.0*n.head(10).reverse();
    for (int i = 1; i < 10; i++)
    companion(i, i-1) = 1.0;
    // Compute eigenvalues. Note, these can be complex.
    Eigen::VectorXcd eigenvalues = companion.eigenvalues();

    // Eigenvalues = z. Plug z back into previous equations to get x, y values.
    std::vector<EssentialMatrix> essential_matrices;
    for (int i = 0; i < eigenvalues.rows(); i++) {
    if (eigenvalues(i).imag() == 0.0) {
    double z = eigenvalues(i).real();
    double x = EvaluatePoly(p1, z)/EvaluatePoly(p3, z);
    double y = EvaluatePoly(p2, z)/EvaluatePoly(p3, z);
    Matrix<double, 9, 1> temp_sum = x*null_space.col(0) +
    y*null_space.col(1) + z*null_space.col(2) + null_space.col(3);
    // Need to do it like this because temp_sum is a row vector and recasting
    // it as a 3x3 will load it column-major.
    Eigen::Matrix3d candidate_essential_mat;
    candidate_essential_mat << temp_sum.head(3).transpose(),
    temp_sum.segment(3, 3).transpose(),
    temp_sum.tail(3).transpose();
    essential_matrices.push_back(candidate_essential_mat);
    }
    }
  */
  return essential_matrices;
}
}  // pose
}  // vision
