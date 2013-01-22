#include "math/matrix/gauss_jordan.h"

#include <Eigen/Dense>
#include "gtest/gtest.h"

namespace math {
namespace matrix {

TEST(GaussJordan, SimpleTest) {
  Eigen::Matrix<double, 3, 6> test_mat;
  test_mat << 0, 2, 1, 1, 0, 0,
      1, 1, 2, 0, 1, 0,
      2, 1, 1, 0, 0, 1;
  Eigen::Matrix3d before_matrix;
  before_matrix << test_mat.col(0), test_mat.col(1), test_mat.col(2);
  GaussJordan(&test_mat);
  Eigen::Matrix3d after_matrix;
  after_matrix << test_mat.col(3), test_mat.col(4), test_mat.col(5);
  ASSERT_TRUE(after_matrix.isApprox(before_matrix.inverse()));
}
}  // namespace matrix
}  // namespace math
