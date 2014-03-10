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

#include "theia/math/polynomial.h"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <glog/logging.h>
#include <complex>
#include <vector>

namespace theia {

namespace {

void BalanceCompanionMatrix(Eigen::MatrixXd* companion) {
  const int degree = companion->rows() - 1;
  Eigen::MatrixXd companion_matrix_offdiagonal = *companion;
  companion_matrix_offdiagonal.diagonal().setZero();

  // gamma <= 1 controls how much a change in the scaling has to
  // lower the 1-norm of the companion matrix to be accepted.
  //
  // gamma = 1 seems to lead to cycles (numerical issues?), so
  // we set it slightly lower.
  const double gamma = 0.9;
  // Greedily scale row/column pairs until there is no change.
  bool scaling_has_changed;
  do {
    scaling_has_changed = false;

    for (int i = 0; i < degree; ++i) {
      const double row_norm =
          companion_matrix_offdiagonal.row(i).template lpNorm<1>();
      const double col_norm =
          companion_matrix_offdiagonal.col(i).template lpNorm<1>();

      // Decompose row_norm/col_norm into mantissa * 2^exponent,
      // where 0.5 <= mantissa < 1. Discard mantissa (return value
      // of frexp), as only the exponent is needed.
      int exponent = 0;
      std::frexp(row_norm / col_norm, &exponent);
      exponent /= 2;

      if (exponent != 0) {
        const double scaled_col_norm = std::ldexp(col_norm, exponent);
        const double scaled_row_norm = std::ldexp(row_norm, -exponent);
        if (scaled_col_norm + scaled_row_norm < gamma * (col_norm + row_norm)) {
          // Accept the new scaling. (Multiplication by powers of 2 should not
          // introduce rounding errors (ignoring non-normalized numbers and
          // over- or underflow))
          scaling_has_changed = true;
          companion_matrix_offdiagonal.row(i) *= std::ldexp(1.0, -exponent);
          companion_matrix_offdiagonal.col(i) *= std::ldexp(1.0, exponent);
        }
      }
    }
  } while (scaling_has_changed);

  companion_matrix_offdiagonal.diagonal() = companion->diagonal();
  *companion = companion_matrix_offdiagonal;
}

}  // namespace

std::vector<std::complex<double> > GetPolynomialRoots(
    const Eigen::VectorXd& coeffs) {
  const int degree = coeffs.rows() - 1;
  Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(degree, degree);

  companion.diagonal(-1).setOnes();
  companion.row(0) = -coeffs.head(degree).reverse();
  companion.row(0) /= coeffs(degree);

  // Compute balancing matrix.
  BalanceCompanionMatrix(&companion);

  // Compute eigenvalues. Note, these can be complex.
  Eigen::VectorXcd eigenvalues = companion.eigenvalues();

  std::vector<std::complex<double> > roots(eigenvalues.rows());
  for (int i = 0; i < roots.size(); i++) {
    roots[i] = eigenvalues(i);
  }
  return roots;
}

std::vector<double> GetRealPolynomialRoots(
    const Eigen::VectorXd& coeffs) {
  std::vector<std::complex<double> > complex_roots = GetPolynomialRoots(coeffs);
  std::vector<double> real_roots;
  real_roots.reserve(complex_roots.size());
  for (const std::complex<double>& root : complex_roots) {
    if (root.imag() == 0.0) {
      real_roots.push_back(root.real());
    }
  }
  return real_roots;
}

}  // namespace theia
