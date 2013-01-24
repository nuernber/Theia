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

#include "math/polynomial.h"

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <vector>
#include "gtest/gtest.h"

namespace math {
using std::vector;
namespace {
// Returns a random double between dMin and dMax
double RandDouble(double dMin, double dMax) {
  double d = static_cast<double>(rand()) / RAND_MAX;
  return dMin + d * (dMax - dMin);
}

void PrintPoly(const Polynomial& poly) {
  std::cout << "Poly = ";
  for (int i = 0; i <= poly.GetDegree(); i++)
    std::cout << std::setw(14) << poly[i];
  std::cout << std::endl;
}
}

TEST(Polynomial, Constructor) {
  int poly_size = rand()%100;
  vector<double> coeffs(poly_size);
  for (int i = 0; i < poly_size; i++)
    coeffs[i] = RandDouble(-1000, 1000);

  Polynomial my_poly(coeffs);
  for (int i = 0; i < coeffs.size(); i++) {
    ASSERT_EQ(coeffs[i], my_poly[i]);
  }
}

TEST(Polynomial, Addition) {
  int poly1_size = rand()%50;
  int poly2_size = rand()%50;
  vector<double> coeffs1(poly1_size);
  vector<double> coeffs2(poly2_size);

  for (int i = 0; i < poly1_size; i++) {
    coeffs1[i] = RandDouble(-1000, 1000);
  }
  for (int i = 0; i < poly2_size; i++) {
    coeffs2[i] = RandDouble(-1000, 1000);
  }

  Polynomial poly1(coeffs1);
  Polynomial poly2(coeffs2);
  Polynomial sum = poly1.Add(poly2);

  // ASSERT Sum is correct.
  int min_size = std::min(poly1.GetDegree() + 1, poly2.GetDegree() + 1);
  for (int i = 0; i < min_size; i++)
    ASSERT_EQ(sum[i], poly1[i] + poly2[i]);

  if (poly1.GetDegree() > poly2.GetDegree()) {
    for (int i = min_size; i <= poly1.GetDegree(); i++)
      ASSERT_EQ(sum[i], poly1[i]);
  } else {
    for (int i = min_size; i <= poly1.GetDegree(); i++)
      ASSERT_EQ(sum[i], poly1[i]);
  }
}

TEST(Polynomial, Subtraction) {
  int poly1_size = rand()%50;
  int poly2_size = rand()%50;
  vector<double> coeffs1(poly1_size);
  vector<double> coeffs2(poly2_size);

  for (int i = 0; i < poly1_size; i++) {
    coeffs1[i] = RandDouble(-1000, 1000);
  }
  for (int i = 0; i < poly2_size; i++) {
    coeffs2[i] = RandDouble(-1000, 1000);
  }

  Polynomial poly1(coeffs1);
  Polynomial poly2(coeffs2);
  Polynomial diff = poly1.Subtract(poly2);

  // ASSERT Diff is correct.
  int min_size = std::min(poly1.GetDegree() + 1, poly2.GetDegree() + 1);
  for (int i = 0; i < min_size; i++)
    ASSERT_EQ(diff[i], poly1[i] - poly2[i]);

  if (poly1.GetDegree() > poly2.GetDegree()) {
    for (int i = min_size; i <= poly1.GetDegree(); i++)
      ASSERT_EQ(diff[i], poly1[i]);
  } else {
    for (int i = min_size; i <= poly1.GetDegree(); i++)
      ASSERT_EQ(diff[i], -poly1[i]);
  }
}

TEST(Polynomial, Multiplication) {
  int poly1_size = rand()%50;
  int poly2_size = rand()%50;
  vector<double> coeffs1(poly1_size);
  vector<double> coeffs2(poly2_size);

  for (int i = 0; i < poly1_size; i++) {
    coeffs1[i] = RandDouble(-10, 10);
  }
  for (int i = 0; i < poly2_size; i++) {
    coeffs2[i] = RandDouble(-10, 10);
  }

  Polynomial poly1(coeffs1);
  Polynomial poly2(coeffs2);
  Polynomial product = poly1.Multiply(poly2);

  // ASSERT that the evaluation of the two polys multiplied together is correct.
  double rand_x = RandDouble(-10, 10);
  ASSERT_DOUBLE_EQ(product.EvalAt(rand_x),
                   poly1.EvalAt(rand_x)*poly2.EvalAt(rand_x));
}


TEST(Polynomial, Division) {
  // Ensure poly2 is of lower degree than poly1
  int poly2_size = rand()%5;
  int poly1_size = poly2_size + rand()%10;
  vector<double> coeffs1(poly1_size);
  vector<double> coeffs2(poly2_size);

  for (int i = 0; i < poly1_size; i++) {
    coeffs1[i] = RandDouble(-10, 10);
  }
  for (int i = 0; i < poly2_size; i++) {
    coeffs2[i] = RandDouble(-10, 10);
  }

  Polynomial poly1(coeffs1);
  Polynomial poly2(coeffs2);
  std::pair<Polynomial, Polynomial> divided = poly1.Divide(poly2);
  Polynomial quotient = divided.first;
  Polynomial remainder = divided.second;

  // ASSERT that the evaluation of the two polys multiplied together is correct.
  double rand_x = RandDouble(-10, 10);
  ASSERT_DOUBLE_EQ(poly1.EvalAt(rand_x),
                   (poly2.Multiply(quotient)).EvalAt(rand_x) +
                   remainder.EvalAt(rand_x));
}

TEST(Polynomial, Differentiate) {
}

TEST(Polynomial, FindRealRoots) {
  double kEps = 1e-6;
  int poly_size = 10;
  vector<double> coeffs(poly_size);
  for (int i = 0; i < poly_size; i++) {
    coeffs[i] = RandDouble(-10, 10);
  }

  // Construct the polynomial as the mulitiplication of all the roots.
  Polynomial my_poly = Polynomial({-coeffs[0], 1.0});
  for (int i = 1; i < poly_size; i++)
    my_poly = my_poly*Polynomial({-coeffs[i], 1.0});

  // Extract real roots.
  std::vector<double> real_roots = my_poly.RealRoots();
  std::sort(coeffs.begin(), coeffs.end());
  for (int i = 0; i < real_roots.size(); i++) {
    ASSERT_LT(fabs(real_roots[i] - coeffs[i]), kEps);
  }
}

}  // namespace math
