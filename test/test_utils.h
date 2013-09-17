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

#ifndef TEST_TEST_UTILS_H_
#define TEST_TEST_UTILS_H_

#include <chrono>
#include <Eigen/Dense>
#include <random>
#include <glog/logging.h>
#include "gtest/gtest.h"

namespace theia {
namespace test {
std::default_random_engine generator;

// Initializes the random generator to be based on the current time. Does not
// have to be called before calling RandDouble, but it works best if it is.
void InitRandomGenerator() {
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  generator.seed(seed);
}

// Get a random double between lower and upper (inclusive).
inline double RandDouble(double lower, double upper) {
  std::uniform_real_distribution<double> distribution(lower, upper);
  return distribution(generator);
}

// Get a random double between lower and upper (inclusive).
inline double RandInt(int lower, int upper) {
  std::uniform_int_distribution<int> distribution(lower, upper);
  return distribution(generator);
}

// Assert that values of the two matrices are nearly the same.
template <typename Derived>
void ExpectMatricesNear(const Eigen::MatrixBase<Derived>& a,
                        const Eigen::MatrixBase<Derived>& b,
                        double tolerance) {
  ASSERT_EQ(a.rows(), b.rows());
  ASSERT_EQ(a.cols(), b.cols());
  for (int i = 0; i < a.rows(); i++)
    for (int j = 0; j < b.rows(); j++)
      ASSERT_NEAR(a(i, j), b(i, j), tolerance);
}

void ExpectArraysNear(int n,
                      const double* a,
                      const double* b,
                      double tolerance) {
  ASSERT_GT(n, 0);
  CHECK(a);
  CHECK(b);
  for (int i = 0; i < n; i++) {
    EXPECT_NEAR(a[i], b[i], tolerance) << "i = " << i;
  }
}

// Expects that for all i = 1,.., n - 1
//
//   |p[i] / max_norm_p - q[i] / max_norm_q| < tolerance
//
// where max_norm_p and max_norm_q are the max norms of the arrays p
// and q respectively.
bool ArraysEqualUpToScale(int n, const double* p, const double* q,
                          double tolerance) {
  CHECK_GT(n, 0);
  CHECK(p);
  CHECK(q);

  double p_max = 0;
  double q_max = 0;
  int p_i = 0;
  int q_i = 0;

  for (int i = 0; i < n; ++i) {
    if (std::abs(p[i]) > p_max) {
      p_max = std::abs(p[i]);
      p_i = i;
    }
    LOG(INFO) << std::abs(q[i]);
    if (std::abs(q[i]) > q_max) {
      q_max = std::abs(q[i]);
      q_i = i;
    }
  }

  // If both arrays are all zeros, they are equal up to scale, but for testing
  // purposes, that's more likely to be an error than a desired result.
  CHECK_NE(p_max, 0.0);
  CHECK_NE(q_max, 0.0);

  for (int i = 0; i < n; ++i) {
    double p_norm = p[i] / p[p_i];
    double q_norm = q[i] / q[q_i];

    if (std::abs(p_norm - q_norm) < tolerance)
      return false;
  }
  return true;
}

}  // namespace test
}  // namespace theia
#endif  // TEST_TEST_UTILS_H_
