#ifndef MATH_MATRIX_GAUSS_JORDAN_H_
#define MATH_MATRIX_GAUSS_JORDAN_H_

#include <Eigen/Core>
#include <algorithm>

namespace math {
namespace matrix {

// Gauss-Jordan elimination on a matrix.
// Modifies the input matrix to be the matrix after gauss-jordan elimation.
template<typename Derived>
void GaussJordan(Eigen::MatrixBase<Derived>* input, int max_rows = 99999) {
  max_rows = std::min(static_cast<int>(input->rows()), max_rows);
  // Iterate through each column one by one.
  for (int i = 0; i < max_rows; i++) {
    // Find row with the largest value in the pivot column and swap.
    int swap_row = i;
    double max_val = 0.0;
    for (int j = i+1; j < input->rows(); j++) {
      double temp_max_val = std::abs((*input)(j, i));
      if (temp_max_val > max_val) {
        max_val = temp_max_val;
        swap_row = j;
      }
    }
    input->row(swap_row).swap(input->row(i));

    // Eliminate all values in the column of the pivot.
    for (int j = 0; j < input->rows(); j++) {
      if (j != i) {
        input->row(j) -= ((*input)(j, i)/(*input)(i, i))*input->row(i);
        (*input)(j, i) = 0.0;
      }
    }
  }

  for (int i = 0; i < max_rows; i++) {
    // Scale current row so that leading value is 1.0. Leading value should be
    // along the diagonal as we proceed with gauss-jordan.
    input->row(i) *= 1.0/(*input)(i, i);
    (*input)(i, i) = 1.0;
  }
}
}  // namespace matrix
}  // namespace math

#endif  // MATH_MATRIX_GAUSS_JORDAN_H_
