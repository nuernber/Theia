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

#include <utility>
#include <vector>

namespace math {
// Class for storing and manipulating polynomials for a univariate
// polynomial.
class Polynomial {
 public:
  // coeffs is an array containing the coefficients of the polynomial, with
  // coeffs[i] corresponding to the i-th degree coefficient.
  Polynomial() {}
  explicit Polynomial(const std::vector<double> coeffs) : coeffs_(coeffs) {}
  Polynomial(const Polynomial& poly) : Polynomial(poly.coeffs_) {}
  Polynomial(int degree, const double coeffs[]);
  ~Polynomial() {}

  // Return the degree of the polynomial.
  int GetDegree() const { return coeffs_.size() - 1; }

  // Operator overload for polynomial access.
  double& operator[](unsigned int i) { return coeffs_[i]; }
  const double& operator[](unsigned int i) const { return coeffs_[i]; }

  // Evaluate the polynomial at x.
  double EvalAt(const double x) const;

  // Add two polynomials together.
  Polynomial Add(const Polynomial& poly) const;
  Polynomial Subtract(const Polynomial& poly) const;
  Polynomial Multiply(const Polynomial& poly) const;
  std::pair<Polynomial, Polynomial> Divide(const Polynomial& poly) const;

  // Operator overload for polynomial arithmatic.
  Polynomial operator+(const Polynomial &poly) { return this->Add(poly); }
  Polynomial operator-(const Polynomial &poly) { return this->Subtract(poly); }
  Polynomial operator*(const Polynomial &poly) { return this->Multiply(poly); }
  // Negation
  Polynomial operator-();

  // First order differentiation.
  Polynomial Differentiate() const;

  // Solve for real roots using Sturm Chains. Note that there are closed-form
  // polynomial solvers for degree up to 4 in
  // math/closed_form_polynomial_solvers.h Additionally, this is only a
  // reasonable solution up to degree 100.
  std::vector<double> RealRoots() const;

 private:
  // Coefficients of the polynomial.
  std::vector<double> coeffs_;
};

}  // namespace math
#endif  // MATH_POLYNOMIAL_H_
