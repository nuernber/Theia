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

#include "math/probability/anderson_darling.h"
#include "gtest/gtest.h"

#include <algorithm>
#include <chrono>
#include <math.h>
#include <random>
#include <vector>

namespace math {
namespace probability {
TEST(Andersondarling, UniformDistribution) {
  int num_pts = 500;
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_real_distribution<double> uniform_distribution(0, num_pts);
  std::vector<double> data;
  for (int i = 0; i < num_pts; i++) {
    double uniform_pt = uniform_distribution(generator);
    data.push_back(uniform_pt);
  }
  std::sort(data.begin(), data.end());
  EXPECT_FALSE(AndersonDarlingTest(data));
}

TEST(Andersondarling, GuassianDistribution) {
  int num_pts = 500;
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<double> gauss_distribution(0, 1.0);
  std::vector<double> data;
  for (int i = 0; i < num_pts; i++) {
    double gauss_pt = gauss_distribution(generator);
    data.push_back(gauss_pt);
  }
  std::sort(data.begin(), data.end());
  EXPECT_TRUE(AndersonDarlingTest(data));
}

TEST(Andersondarling, MixedDistribution) {
  int num_pts = 1000;
  double inlier_ratio = 0.4;
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::normal_distribution<double> gauss_distribution(0, 1.0);
  std::uniform_real_distribution<double>
      uniform_distribution(0, inlier_ratio*num_pts);
  std::vector<double> data;
  for (int i = 0; i < inlier_ratio*num_pts; i++) {
    double gauss_pt = gauss_distribution(generator);
    data.push_back(gauss_pt*gauss_pt);
  }
  for (int i = 0; i < inlier_ratio*(1.0 - num_pts); i++) {
    double uniform_pt = uniform_distribution(generator);
    data.push_back(uniform_pt);
  }
  std::sort(data.begin(), data.end());
  
  EXPECT_TRUE(AndersonDarlingTest(data));
  // Test for different values of n.
  for (int n = 50; n < inlier_ratio*num_pts; n += 50) {
    EXPECT_TRUE(AndersonDarlingTest(data, n));
  }
}


}  // namespace probability
}  // namespace math
