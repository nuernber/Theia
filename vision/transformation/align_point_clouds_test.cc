#include "gtest/gtest.h"
#include "vision/transformation/align_point_clouds.h"
#include <cmath>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

namespace vision {
namespace transformation {
namespace {
double kEpsilon = 1e-6;
}  // namespace
TEST(AlignPointClouds, SimpleTransform) {
  // Make the model a 3x3 grid.
  Matrix<double, 9, 3, RowMajor> left_mat;
  left_mat << 0.4, -3.105, 2.147,
      1.293, 7.1982, -.068,
      -5.34, 0.708, -3.69,
      -.345, 1.987, 0.936,
      0.93, 1.45, 1.079,
      -3.15, -4.73, 2.49,
      2.401, -2.03, -1.87,
      3.192, -.573, 0.1,
      -2.53, 3.07, -5.19;
      
  // Reference the matrix as a 2d array.
  double (*left)[3] = (double (*)[3])left_mat.data();
  
  // Shift and rotate the points.
  Matrix3d rotation_mat;
  rotation_mat << 0, 0, 1.0,
      -1.0, 0, 0,
      0, -1.0, 0;
  Vector3d translation_vec(0, 2, 2);

  // Rotate the left points.
  Matrix<double, 3, 9> left_mat_transformed =
      rotation_mat*(left_mat.transpose());
  // Translate the left points.
  for (int i = 0; i < 9; i++)
    left_mat_transformed.col(i) += translation_vec;

  // Create a 2d accessor for the transformed points.
  double (*right)[3] = (double (*)[3])left_mat_transformed.data();
  
  double rotation[3][3];
  double translation[3];

  // Map the left points to the right points
  AlignPointClouds(left, right, 9, rotation, translation);
  
  // Ensure the calculated transformation is the same as the one we set.
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      ASSERT_LT(abs(rotation[i][j] - rotation_mat(i,j)), kEpsilon);
    }
    ASSERT_LT(abs(translation[i] - translation_vec(i)), kEpsilon);
  }
}

}  // namespace transformation
}  // namespace vision
