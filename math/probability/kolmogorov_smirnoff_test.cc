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
// ARE DISCLAIMED. IN NO EVENT SHALL CHRIS SWEENEY BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#include "math/probability/kolmogorov_smirnoff.h"
#include "gtest/gtest.h"

#include <algorithm>
#include <chrono>
#include <math.h>
#include <random>
#include <vector>

namespace math {
namespace probability {
TEST(KolmogorovSmirnoff, SameDistributionGaussian) {
  int num_pts = 500;
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<double> gauss_distribution(0.0, 1.0);
  std::vector<double> data1;
  std::vector<double> data2;
  for (int i = 0; i < num_pts; i++) {
    double gauss_pt = gauss_distribution(generator);
    data1.push_back(gauss_pt*gauss_pt);

    double gauss_pt2 = gauss_distribution(generator);
    data2.push_back(gauss_pt2*gauss_pt2);
  }
  std::sort(data1.begin(), data1.end());
  std::sort(data2.begin(), data2.end());

  // These two distros should definitely be equal!
  EXPECT_TRUE(KolmogorovSmirnoffTest(data1, data2));
  
  // test for several values of n, all of which should be true.
  for (int n = num_pts/10; n < num_pts; n += num_pts/10)
    EXPECT_TRUE(KolmogorovSmirnoffTest(data1, data2, n));
}

TEST(KolmogorovSmirnoff, SameDistributionUniform) {
  int num_pts = 500;
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution<double> uniform_distribution(0, num_pts);
  std::vector<double> data1;
  std::vector<double> data2;
  for (int i = 0; i < num_pts; i++) {
    double uniform_pt = uniform_distribution(generator);
    data1.push_back(uniform_pt*uniform_pt);

    double uniform_pt2 = uniform_distribution(generator);
    data2.push_back(uniform_pt2*uniform_pt2);
  }
  std::sort(data1.begin(), data1.end());
  std::sort(data2.begin(), data2.end());

  // These two distros should definitely be equal!
  EXPECT_TRUE(KolmogorovSmirnoffTest(data1, data2));

  // test for several values of n, all of which should be true.
  for (int n = num_pts/10; n < num_pts; n += num_pts/10)
    EXPECT_TRUE(KolmogorovSmirnoffTest(data1, data2, n));
}

TEST(KolmogorovSmirnoff, DifferentDistribution) {
  int num_pts = 500;
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<double> gauss_distribution(0, 1.0);
  std::uniform_real_distribution<double> uniform_distribution(0, num_pts);
  std::vector<double> data1;
  std::vector<double> data2;
  for (int i = 0; i < num_pts; i++) {
    double gauss_pt = gauss_distribution(generator);
    data1.push_back(gauss_pt*gauss_pt);

    double uniform_pt = uniform_distribution(generator);
    data2.push_back(uniform_pt*uniform_pt);
  }
  std::sort(data1.begin(), data1.end());
  std::sort(data2.begin(), data2.end());

  // These two distros should not be equal!
  EXPECT_FALSE(KolmogorovSmirnoffTest(data1, data2));

  // test for several values of n, all of which should be false!
  for (int n = num_pts/10; n < num_pts; n += num_pts/10)
    EXPECT_FALSE(KolmogorovSmirnoffTest(data1, data2, n));
}

}  // namespace probability
}  // namespace math
