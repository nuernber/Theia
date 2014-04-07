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

// These functions are largely based off the the Ceres solver polynomial
// functions which are not available through the public interface. The license
// is below:
//
// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2012 Google Inc. All rights reserved.
// http://code.google.com/p/ceres-solver/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: moll.markus@arcor.de (Markus Moll)
//         sameeragarwal@google.com (Sameer Agarwal)

#include <glog/logging.h>
#include <algorithm>
#include <vector>
#include "gtest/gtest.h"

#include "theia/math/polynomial.h"
#include "theia/test/test_utils.h"

namespace theia {

namespace {

// For IEEE-754 doubles, machine precision is about 2e-16.
const double kEpsilon = 1e-13;
const double kEpsilonLoose = 1e-9;

// Return the constant polynomial p(x) = 1.23.
Eigen::VectorXd ConstantPolynomial(double value) {
  Eigen::VectorXd poly(1);
  poly(0) = value;
  return poly;
}

// Return the polynomial p(x) = poly(x) * (x - root).
Eigen::VectorXd AddRealRoot(const Eigen::VectorXd& poly, double root) {
  Eigen::VectorXd poly2(poly.size() + 1);
  poly2.setZero();
  poly2.head(poly.size()) += poly;
  poly2.tail(poly.size()) -= root * poly;
  return poly2;
}

// Return the polynomial
// p(x) = poly(x) * (x - real - imag*i) * (x - real + imag*i).
Eigen::VectorXd AddComplexRootPair(const Eigen::VectorXd& poly, double real,
                                   double imag) {
  Eigen::VectorXd poly2(poly.size() + 2);
  poly2.setZero();
  // Multiply poly by x^2 - 2real + abs(real,imag)^2
  poly2.head(poly.size()) += poly;
  poly2.segment(1, poly.size()) -= 2 * real * poly;
  poly2.tail(poly.size()) += (real * real + imag * imag) * poly;
  return poly2;
}

// Sort the entries in a vector.
// Needed because the roots are not returned in sorted order.
Eigen::VectorXd SortVector(const Eigen::VectorXd& in) {
  Eigen::VectorXd out(in);
  std::sort(out.data(), out.data() + out.size());
  return out;
}

// Run a test with the polynomial defined by the N real roots in roots_real.
// If use_real is false, NULL is passed as the real argument to
// FindPolynomialRoots. If use_imaginary is false, NULL is passed as the
// imaginary argument to FindPolynomialRoots.
template <int N>
void RunPolynomialTestRealRoots(const double (&real_roots)[N], bool use_real,
                                bool use_imaginary, double epsilon) {
  Eigen::VectorXd real;
  Eigen::VectorXd imaginary;
  Eigen::VectorXd poly = ConstantPolynomial(1.23);
  for (int i = 0; i < N; ++i) {
    poly = AddRealRoot(poly, real_roots[i]);
  }
  Eigen::VectorXd* const real_ptr = use_real ? &real : NULL;
  Eigen::VectorXd* const imaginary_ptr = use_imaginary ? &imaginary : NULL;
  bool success = FindPolynomialRoots(poly, real_ptr, imaginary_ptr);

  EXPECT_EQ(success, true);
  if (use_real) {
    EXPECT_EQ(real.size(), N);
    real = SortVector(real);
    test::ExpectArraysNear(N, real.data(), real_roots, epsilon);
  }
  if (use_imaginary) {
    EXPECT_EQ(imaginary.size(), N);
    const Eigen::VectorXd zeros = Eigen::VectorXd::Zero(N);
    test::ExpectArraysNear(N, imaginary.data(), zeros.data(), epsilon);
  }
}
}  // namespace

TEST(Polynomial, InvalidPolynomialOfZeroLengthIsRejected) {
  // Vector poly(0) is an ambiguous constructor call, so
  // use the constructor with explicit column count.
  Eigen::VectorXd poly(0, 1);
  Eigen::VectorXd real;
  Eigen::VectorXd imag;
  bool success = FindPolynomialRoots(poly, &real, &imag);

  EXPECT_EQ(success, false);
}

TEST(Polynomial, ConstantPolynomialReturnsNoRoots) {
  Eigen::VectorXd poly = ConstantPolynomial(1.23);
  Eigen::VectorXd real;
  Eigen::VectorXd imag;
  bool success = FindPolynomialRoots(poly, &real, &imag);

  EXPECT_EQ(success, true);
  EXPECT_EQ(real.size(), 0);
  EXPECT_EQ(imag.size(), 0);
}

TEST(Polynomial, LinearPolynomialWithPositiveRootWorks) {
  const double roots[1] = { 42.42 };
  RunPolynomialTestRealRoots(roots, true, true, kEpsilon);
}

TEST(Polynomial, LinearPolynomialWithNegativeRootWorks) {
  const double roots[1] = { -42.42 };
  RunPolynomialTestRealRoots(roots, true, true, kEpsilon);
}

TEST(Polynomial, QuadraticPolynomialWithPositiveRootsWorks) {
  const double roots[2] = { 1.0, 42.42 };
  RunPolynomialTestRealRoots(roots, true, true, kEpsilon);
}

TEST(Polynomial, QuadraticPolynomialWithOneNegativeRootWorks) {
  const double roots[2] = { -42.42, 1.0 };
  RunPolynomialTestRealRoots(roots, true, true, kEpsilon);
}

TEST(Polynomial, QuadraticPolynomialWithTwoNegativeRootsWorks) {
  const double roots[2] = { -42.42, -1.0 };
  RunPolynomialTestRealRoots(roots, true, true, kEpsilon);
}

TEST(Polynomial, QuadraticPolynomialWithCloseRootsWorks) {
  const double roots[2] = { 42.42, 42.43 };
  RunPolynomialTestRealRoots(roots, true, false, kEpsilonLoose);
}

TEST(Polynomial, QuadraticPolynomialWithComplexRootsWorks) {
  Eigen::VectorXd real;
  Eigen::VectorXd imag;

  Eigen::VectorXd poly = ConstantPolynomial(1.23);
  poly = AddComplexRootPair(poly, 42.42, 4.2);
  bool success = FindPolynomialRoots(poly, &real, &imag);

  EXPECT_EQ(success, true);
  EXPECT_EQ(real.size(), 2);
  EXPECT_EQ(imag.size(), 2);
  EXPECT_NEAR(real(0), 42.42, kEpsilon);
  EXPECT_NEAR(real(1), 42.42, kEpsilon);
  EXPECT_NEAR(std::abs(imag(0)), 4.2, kEpsilon);
  EXPECT_NEAR(std::abs(imag(1)), 4.2, kEpsilon);
  EXPECT_NEAR(std::abs(imag(0) + imag(1)), 0.0, kEpsilon);
}

TEST(Polynomial, QuarticPolynomialWorks) {
  const double roots[4] = { 1.23e-4, 1.23e-1, 1.23e+2, 1.23e+5 };
  RunPolynomialTestRealRoots(roots, true, true, kEpsilon);
}

TEST(Polynomial, QuarticPolynomialWithTwoClustersOfCloseRootsWorks) {
  const double roots[4] = { 1.23e-1, 2.46e-1, 1.23e+5, 2.46e+5 };
  RunPolynomialTestRealRoots(roots, true, true, kEpsilonLoose);
}

TEST(Polynomial, QuarticPolynomialWithTwoZeroRootsWorks) {
  const double roots[4] = { -42.42, 0.0, 0.0, 42.42 };
  RunPolynomialTestRealRoots(roots, true, true, kEpsilonLoose);
}

TEST(Polynomial, QuarticMonomialWorks) {
  const double roots[4] = { 0.0, 0.0, 0.0, 0.0 };
  RunPolynomialTestRealRoots(roots, true, true, kEpsilon);
}

TEST(Polynomial, NullPointerAsImaginaryPartWorks) {
  const double roots[4] = { 1.23e-4, 1.23e-1, 1.23e+2, 1.23e+5 };
  RunPolynomialTestRealRoots(roots, true, false, kEpsilon);
}

TEST(Polynomial, NullPointerAsRealPartWorks) {
  const double roots[4] = { 1.23e-4, 1.23e-1, 1.23e+2, 1.23e+5 };
  RunPolynomialTestRealRoots(roots, false, true, kEpsilon);
}

TEST(Polynomial, BothOutputArgumentsNullWorks) {
  const double roots[4] = { 1.23e-4, 1.23e-1, 1.23e+2, 1.23e+5 };
  RunPolynomialTestRealRoots(roots, false, false, kEpsilon);
}

TEST(Polynomial, DifferentiateConstantPolynomial) {
  // p(x) = 1;
  Eigen::VectorXd polynomial(1);
  polynomial(0) = 1.0;
  const Eigen::VectorXd derivative = DifferentiatePolynomial(polynomial);
  EXPECT_EQ(derivative.rows(), 1);
  EXPECT_EQ(derivative(0), 0);
}

TEST(Polynomial, DifferentiateQuadraticPolynomial) {
  // p(x) = x^2 + 2x + 3;
  Eigen::VectorXd polynomial(3);
  polynomial(0) = 1.0;
  polynomial(1) = 2.0;
  polynomial(2) = 3.0;

  const Eigen::VectorXd derivative = DifferentiatePolynomial(polynomial);
  EXPECT_EQ(derivative.rows(), 2);
  EXPECT_EQ(derivative(0), 2.0);
  EXPECT_EQ(derivative(1), 2.0);
}

TEST(Polynomial, MinimizeConstantPolynomial) {
  // p(x) = 1;
  Eigen::VectorXd polynomial(1);
  polynomial(0) = 1.0;

  double optimal_x = 0.0;
  double optimal_value = 0.0;
  double min_x = 0.0;
  double max_x = 1.0;
  MinimizePolynomial(polynomial, min_x, max_x, &optimal_x, &optimal_value);

  EXPECT_EQ(optimal_value, 1.0);
  EXPECT_LE(optimal_x, max_x);
  EXPECT_GE(optimal_x, min_x);
}

TEST(Polynomial, MinimizeLinearPolynomial) {
  // p(x) = x - 2
  Eigen::VectorXd polynomial(2);

  polynomial(0) = 1.0;
  polynomial(1) = 2.0;

  double optimal_x = 0.0;
  double optimal_value = 0.0;
  double min_x = 0.0;
  double max_x = 1.0;
  MinimizePolynomial(polynomial, min_x, max_x, &optimal_x, &optimal_value);

  EXPECT_EQ(optimal_x, 0.0);
  EXPECT_EQ(optimal_value, 2.0);
}

TEST(Polynomial, MinimizeQuadraticPolynomial) {
  // p(x) = x^2 - 3 x + 2
  // min_x = 3/2
  // min_value = -1/4;
  Eigen::VectorXd polynomial(3);
  polynomial(0) = 1.0;
  polynomial(1) = -3.0;
  polynomial(2) = 2.0;

  double optimal_x = 0.0;
  double optimal_value = 0.0;
  double min_x = -2.0;
  double max_x = 2.0;
  MinimizePolynomial(polynomial, min_x, max_x, &optimal_x, &optimal_value);
  EXPECT_EQ(optimal_x, 3.0 / 2.0);
  EXPECT_EQ(optimal_value, -1.0 / 4.0);

  min_x = -2.0;
  max_x = 1.0;
  MinimizePolynomial(polynomial, min_x, max_x, &optimal_x, &optimal_value);
  EXPECT_EQ(optimal_x, 1.0);
  EXPECT_EQ(optimal_value, 0.0);

  min_x = 2.0;
  max_x = 3.0;
  MinimizePolynomial(polynomial, min_x, max_x, &optimal_x, &optimal_value);
  EXPECT_EQ(optimal_x, 2.0);
  EXPECT_EQ(optimal_value, 0.0);
}

}  // namespace theia
