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

#ifndef MATH_POLYNOMIAL_H_
#define MATH_POLYNOMIAL_H_

#include <Eigen/Dense>
#include <glog/logging.h>
#include <tuple>

#include <algorithm>
#include <complex>
#include <glog/logging.h>
#include <stdlib.h>
#include <utility>
#include <vector>

namespace math {
// Class for storing and manipulating polynomials for a univariate
// polynomial.
template<int degree>
class Polynomial {
 public:
  // coeffs is an array containing the coefficients of the polynomial, with
  // coeffs[i] corresponding to the i-th degree coefficient.
  Polynomial() : coeffs_(Eigen::Matrix<double, degree + 1, 1>::Zero()) {}
  
  explicit Polynomial(const std::vector<double> coeffs) {
    std::copy(coeffs.begin(), coeffs.end(), coeffs_.data());
  }
  
  Polynomial(const Polynomial& poly) : coeffs_(poly.coeffs_) {}

  explicit Polynomial(const double coeffs[]) {
    std::copy(coeffs, coeffs + degree + 1, coeffs_.data());
  }

  ~Polynomial() {}

  // Return the degree of the polynomial.
  int GetDegree() const { return degree; }

  // Operator overload for polynomial access.
  double& operator[](unsigned int i) { return coeffs_[i]; }
  const double& operator[](unsigned int i) const { return coeffs_[i]; }

  // Evaluate the polynomial at x.
  double EvalAt(const double x) const {
    double val = 0;
    for (int i = degree; i > 0; i--) {
      val += coeffs_[i];
      val *= x;
    }
    val += coeffs_[0];
    return val;
  }

  // Add two polynomials together.
  template<int degree2>
  Polynomial Add(const Polynomial<degree2>& poly) const {
    CHECK_EQ(degree, degree2) << "Cannot add polynomials of different sizes";
    Polynomial<degree> sum;
    sum.coeffs_ = coeffs_ + poly.coeffs_;
    return sum;
  }

  // Subtract two polynomials.
  template<int degree2>
  Polynomial Subtract(const Polynomial<degree2>& poly) const {
    CHECK_EQ(degree, degree2) << "Cannot subtract polynomials of different "
                              << "sizes";
    Polynomial<degree> diff;
    diff.coeffs_ = coeffs_ - poly.coeffs_;
    return diff;
  }

  // Multiply two polynomials.
  template<int degree2>
  Polynomial<degree + degree2> Multiply(const Polynomial<degree2>& poly) const {
    Polynomial<degree + degree2> product;
    for (int i = 0; i <= degree; i++)
      for (int j = 0; j <= degree2; j++)
        product.coeffs_[i + j] += coeffs_[i]*poly.coeffs_[j];
    return product;
  }

  // Divide two polynomials, returning the quotient and remainder.
  // TODO(cmsweeney): Make this more efficient with Eigen.
  template<int degree2>
  std::pair<Polynomial<degree - degree2>, Polynomial<degree2 - 1> > Divide(
      const Polynomial<degree2>& poly) const {
    Polynomial<degree - degree2> quotient;
    Eigen::Matrix<double, degree + 1, 1> remainder(coeffs_);

    for (int i = quotient.GetDegree(); i >= 0; i--) {
      double scale = remainder[i + poly.coeffs_.size() - 1]/
          poly.coeffs_[poly.coeffs_.size() - 1];
      quotient.coeffs_[i] = scale;
      for (int j = 0; j < poly.coeffs_.size(); j++)
        remainder[i + j] -= scale*poly.coeffs_[j];
    }
    return std::make_pair(
        quotient,
        Polynomial<degree2 - 1>(remainder.head(degree2).data()));
  }

  // Operator overload for polynomial arithmatic.
  template<int degree2>
  Polynomial<degree> operator+(const Polynomial<degree2> &poly) {
    return this->Add(poly);
  }
  
  template<int degree2>
  Polynomial<degree> operator-(const Polynomial<degree> &poly) {
    return this->Subtract(poly);
  }

  template<int degree2>
  Polynomial<degree + degree2> operator*(const Polynomial<degree2> &poly) {
    return this->Multiply(poly);
  }

  // Negation
  Polynomial<degree> operator-() { return Polynomail(-coeffs_); }

  // First order differentiation.
  Polynomial<degree - 1> Differentiate() const {
    Polynomial<degree - 1> derivitive = coeffs_.tail(degree);
    for (int i = 0; i < degree; i++)
      derivitive[i] *= i + 1;
    return derivitive;
  }

  // Returns all real roots to the polynomial.
  std::vector<double> RealRoots() const {
    std::vector<std::complex<double> > complex_roots = Roots();
    std::vector<double> roots;
    double kEps = 1e-12;
    for (int i = 0; i < complex_roots.size(); i++) {
      if (fabs(complex_roots[i].imag()) < kEps)
        roots.push_back(complex_roots[i].real());
    }
    return roots;
  }

  // Finds all roots (real and complex) of the polynomial by building a
  // companion matrix and finding the eigenvalues of that matrix. The companion
  // matrix is first balanced (rearranged and scaled so that there are fewer
  // changes in orders of magnitude amongst the matrix entries) to increase the
  // stability of the eigenvalues, which are otherwise sensitive.
  std::vector<std::complex<double> > Roots() const {
    Eigen::Matrix<double, degree, degree> companion =
        Eigen::Matrix<double, degree, degree>::Zero();

    companion.diagonal(-1).setOnes();
    companion.row(0) = -coeffs_.head(degree).reverse();
    companion.row(0) /= coeffs_(degree);

    // Compute balancing matrix.
    BalanceCompanionMatrix(&companion);

    // Compute eigenvalues. Note, these can be complex.
    Eigen::VectorXcd eigenvalues = companion.eigenvalues();

    std::vector<std::complex<double> > roots(eigenvalues.rows());
    for (int i = 0; i < roots.size(); i++)
      roots[i] = eigenvalues(i);
    return roots;
  }

 private:
  // Coefficients of the polynomial.
  Eigen::Matrix<double, degree + 1, 1> coeffs_;

  template<int friend_degree>
  friend class Polynomial;

  // Borrowed from Ceres Solver, implementing balancing function as described by
  // B. N. Parlett and C. Reinsch, "Balancing a Matrix for Calculation of
  // Eigenvalues and Eigenvectors".  In: Numerische Mathematik, Volume 13,
  // Number 4 (1969), 293-304, Springer Berlin Heidelberg. DOI:
  // 10.1007/BF02165404
  void BalanceCompanionMatrix(
      Eigen::Matrix<double, degree, degree>* companion) const {
    Eigen::Matrix<double, degree, degree>& companion_matrix =
        *companion;
    Eigen::Matrix<double, degree, degree> companion_matrix_offdiagonal =
        companion_matrix;
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
          if (scaled_col_norm + scaled_row_norm < gamma*(col_norm + row_norm)) {
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

    companion_matrix_offdiagonal.diagonal() = companion_matrix.diagonal();
    companion_matrix = companion_matrix_offdiagonal;
  }
};

}  // namespace math
#endif  // MATH_POLYNOMIAL_H_
