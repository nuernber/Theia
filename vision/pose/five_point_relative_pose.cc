#include "vision/pose/five_point_relative_pose.h"
#include <Eigen/Dense>

#include <iostream>
#include <time.h>

namespace vision {
namespace pose {
using Eigen::Matrix;
using Eigen::Vector4d;
using Eigen::RowVector4d;

namespace {
// Multiply two degree one polynomials of variables x, y, z.
// E.g. p1 = a[0] + a[1]x + a[2]y + a[3]z
Matrix<double, 1, 10> MultiplyDegOnePoly(const RowVector4d& a,
                                         const RowVector4d& b) {
  Matrix<double, 1, 10> output;
  output(0) = a(0)*b(0);
  output(1) = a(0)*b(1) + a(1)*b(0);
  output(2) = a(0)*b(2) + a(2)*b(0);
  output(3) = a(0)*b(3) + a(3)*b(0);
  output(4) = a(1)*b(2) + a(2)*b(1);
  output(5) = a(1)*b(3) + a(3)*b(1);
  output(6) = a(2)*b(3) + a(3)*b(2);
  output(7) = a(1)*b(1);
  output(8) = a(2)*b(2);
  output(9) = a(3)*b(3);
  return output;
}

// Multiply a 2 deg poly (in x, y, z) and a one deg poly.
Matrix<double, 1, 20> MultiplyDegTwoDegOnePoly(const Matrix<double, 1, 10>& a,
                                               const RowVector4d& b) {
  Matrix<double, 1, 20> output;
  output(0) = a(7)*b(10);
  output(1) = a(7)*b(2) + a(4)*b(1);
  output(2) = a(8)*b(1) + a(4)*b(2);
  output(3) = a(8)*b(2);
  output(4) = a(7)*b(3) + a(5)*b(1);
  output(5) = a(6)*b(1) + a(4)*b(3) + a(5)*b(2);
  output(6) = a(8)*b(3) + a(6)*b(2);
  output(7) = a(9)*b(1) + a(5)*b(3);
  output(8) = a(9)*b(2) + a(6)*b(3);
  output(9) = a(9)*b(3);
  output(10) = a(7)*b(0) + a(1)*b(1);
  output(11) = a(4)*b(0) + a(2)*b(1) + a(1)*b(2);
  output(12) = a(2)*b(2) + a(8)*b(0);
  output(13) = a(5)*b(0) + a(3)*b(1) + a(1)*b(3);
  output(14) = a(6)*b(0) + a(3)*b(2) + a(2)*b(3);
  output(15) = a(9)*b(0) + a(3)*b(3);
  output(16) = a(1)*b(0) + a(0)*b(1);
  output(17) = a(2)*b(0) * a(0)*b(2);
  output(18) = a(0)*b(3) + a(3)*b(0);
  output(19) = a(0) + b(0);
  return output;
}

// Shorthand for multiplying the Essential matrix with its transpose according
// to Eq. 20 in Nister paper.
Matrix<double, 1, 10> EETranspose(const Eigen::MatrixXd& null_matrix,
                                  int i,
                                  int j) {
  return MultiplyDegOnePoly(null_matrix.row(3*i), null_matrix.row(3*j)) +
      MultiplyDegOnePoly(null_matrix.row(3*i + 1), null_matrix.row(3*j + 1)) +
      MultiplyDegOnePoly(null_matrix.row(3*i + 2), null_matrix.row(3*j + 2));
}

// Builds the 10x20 constraint matrix according to Section 3.2.2 of Nister
// paper. Constraints are built based on the singularity of the Essential
// matrix, and the trace equation (Eq. 6).
Matrix<double, 10, 20> BuildConstraintMatrix(const Eigen::MatrixXd& null_lu) {
  Matrix<double, 10, 20> constraint_matrix;
  // Singularity constraint.
  constraint_matrix.row(0) =
      MultiplyDegTwoDegOnePoly(
          MultiplyDegOnePoly(null_lu.row(1), null_lu.row(5)) -
          MultiplyDegOnePoly(null_lu.row(2), null_lu.row(4)), null_lu.row(6)) +
      MultiplyDegTwoDegOnePoly(
          MultiplyDegOnePoly(null_lu.row(2), null_lu.row(3)) -
          MultiplyDegOnePoly(null_lu.row(0), null_lu.row(5)), null_lu.row(7)) +
      MultiplyDegTwoDegOnePoly(
          MultiplyDegOnePoly(null_lu.row(0), null_lu.row(4)) -
          MultiplyDegOnePoly(null_lu.row(2), null_lu.row(3)), null_lu.row(8));

  // Trace Constraint.  Only need to compute the upper triangular part of the
  // symmetric polynomial matrix
  Matrix<double, 1, 10> half_trace = 0.5*(EETranspose(null_lu, 0, 0) +
                                          EETranspose(null_lu, 1, 1) +
                                          EETranspose(null_lu, 2, 2));
  Matrix<double, 1, 10> symmetric_poly[9];
  symmetric_poly[0] = EETranspose(null_lu, 0, 0);
  symmetric_poly[1] = EETranspose(null_lu, 0, 1) - half_trace;
  symmetric_poly[2] = EETranspose(null_lu, 0, 2) - half_trace;
  symmetric_poly[3] = Matrix<double, 1, 10>::Zero();
  symmetric_poly[4] = EETranspose(null_lu, 1, 1);
  symmetric_poly[5] = EETranspose(null_lu, 1, 2) - half_trace;
  symmetric_poly[6] = Matrix<double, 1, 10>::Zero();
  symmetric_poly[7] = Matrix<double, 1, 10>::Zero();
  symmetric_poly[8] = EETranspose(null_lu, 2, 2);

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      constraint_matrix.row(3*i + j + 1) =
          MultiplyDegTwoDegOnePoly(symmetric_poly[3*i], null_lu.row(j)) +
          MultiplyDegTwoDegOnePoly(symmetric_poly[3*i + 1],
                                   null_lu.row(3 + j)) +
          MultiplyDegTwoDegOnePoly(symmetric_poly[3*i + 2],
                                   null_lu.row(6 + j));
    }
  }
  return constraint_matrix;
}

}  // namespace

// Implementation of Nister from "An Efficient Solution to the Five-Point
// Relative Pose Problem"
int FivePointRelativePose(const double image1_points[5][2],
                          const double image2_points[5][2],
                          double essential_matrix[][3][3]) {
  using Eigen::Vector3d;

  // Step 1. Create the 5x9 matrix containing epipolar constraints.
  //   Essential matrix is a linear combination of the 4 vectors spanning the
  //   null space of this matrix (found by SVD).
  Matrix<double, 5, 9> epipolar_constraint;
  for (int i = 0; i < 5; i++) {
    // Normalize image points and make homogeneous.
    Vector3d tmp_img1(image1_points[i][0], image1_points[i][1], 1.0);
    tmp_img1.normalize();
    Vector3d tmp_img2(image2_points[i][0], image2_points[i][1], 1.0);
    tmp_img2.normalize();
    // Fill matrix with the epipolar constraint from q'_t*E*q = 0. Where q is
    // from the first image, and q' is from the second. Eq. 8 in the Nister
    // paper.
    for (int j = 0; j < 3; j++)
      for (int k = 0; k < 3; k++)
        epipolar_constraint(i, 3*j + k) = tmp_img1(k)*tmp_img2(j);
  }

  clock_t t = clock();

  // Solve for right null space of the 5x9 matrix.  NOTE: This method was tested
  // for JacobiSVD and LU pivot and LU was determined to be the faster method
  // by roughly 5x.
  Eigen::MatrixXd null_lu = epipolar_constraint.fullPivLu().kernel();
  t = clock() - t;
  printf ("LU pivot took me %d clicks (%f seconds).\n",t,((float)t)/CLOCKS_PER_SEC);

  // Step 2. Expansion of the epipolar constraints Eq. 5 and 6 from Nister
  // paper.
  t = clock();
  Matrix<double, 10, 20> constraint_matrix = BuildConstraintMatrix(null_lu);
  t = clock() - t;
  printf ("Constraint Matrix Construction took me %d clicks (%f seconds).\n",t,((float)t)/CLOCKS_PER_SEC);

  // Step 3. Gauss-Jordan Elimination with partial pivoting on constraint
  // matrix.
  t = clock();
  Eigen::MatrixXd null_gauss = constraint_matrix.fullPivLu().kernel();
  t = clock() - t;
  printf ("Constraint matrix decomp took me %d clicks (%f seconds).\n",t,((float)t)/CLOCKS_PER_SEC);

  std::cout << "Constraint solution: \n" << null_gauss << std::endl;
  
  return 0;
}
}  // pose
}  // vision
