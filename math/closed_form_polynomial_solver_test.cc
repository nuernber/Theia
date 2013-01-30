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

#include <complex>
#include "gtest/gtest.h"
#include "math/closed_form_polynomial_solver.h"

using std::complex;

namespace math {

TEST(SolveQuadraticPolynomial, DegenerateSolution) {
  // - 2x + 1 = 0
  double a = 0.0;
  double b = -2.0;
  double c = 1.0;
  double roots[2];
  int num_roots = SolveQuadraticReals(a, b, c, roots);
  EXPECT_EQ(num_roots, 1);
  EXPECT_EQ(roots[0], 0.5);
}

TEST(SolveQuadraticPolynomial, SolveReals) {
  // x^2 - 2x + 1 = 0
  double a = 1.0;
  double b = -2.0;
  double c = 1.0;
  double roots[2];
  int num_roots = SolveQuadraticReals(a, b, c, roots);
  EXPECT_EQ(num_roots, 2);
  EXPECT_EQ(roots[0], 1.0);

  // x^2 - 11x + 30 = 0
  a = 1.0;
  b = -11.0;
  c = 30.0;
  num_roots = SolveQuadraticReals(a, b, c, roots);
  EXPECT_EQ(num_roots, 2);
  EXPECT_NE(roots[0], roots[1]);
  EXPECT_TRUE(roots[0] == 5.0 || roots[1] == 5.0);
  EXPECT_TRUE(roots[0] == 6.0 || roots[1] == 6.0);
}

TEST(SolveQuadraticPolynomial, SolveComplex) {
  // x^2 - 2x + 5 = 0 should yield 1 + 2i, 1 - 2i
  double a = 1.0;
  double b = -2.0;
  double c = 5.0;
  complex<double> roots[2];
  int num_roots = SolveQuadratic(a, b, c, roots);
  EXPECT_EQ(num_roots, 2);
  EXPECT_EQ(roots[0].real(), 1.0);
  EXPECT_EQ(roots[1].real(), 1.0);
  EXPECT_TRUE(roots[0].imag() == 2.0 || roots[0].imag() == -2.0);
  EXPECT_TRUE(roots[1].imag() == 2.0 || roots[1].imag() == -2.0);
}

TEST(SolveCubicPolynomial, SolveReals) {
  // x^3 - 6x^2 + 11x - 6 = 0
  double a = 1.0;
  double b = -6.0;
  double c = 11.0;
  double d = -6.0;
  double roots[3];
  int num_roots = SolveCubicReals(a, b, c, d, roots);
  std::cout << "num roots: " << num_roots << std::endl;
  for (int i = 0; i < 3; i++)
    std::cout << roots[i] << std::endl;
}

TEST(SolveQuarticPolynomial, SolveReals) {
  // y = 3x^4 + 6x^3 - 123x^2 - 126x + 1080 = 0
  double a = 3.0;
  double b = 6.0;
  double c = -123.0;
  double d = -126.0;
  double e = 1080.0;

  double roots[4];
  int num_roots = SolveQuarticReals(a, b, c, d, e, roots);
  std::cout << "num roots: " << num_roots << std::endl;
  for (int i = 0; i < 4; i++)
    std::cout << roots[i] << std::endl;
}
}  // namespace math
