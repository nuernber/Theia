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

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <bitset>
#include <string>
#include "gtest/gtest.h"

#include "theia/test/benchmark.h"
#include "theia/util/random.h"
#include "theia/vision/matching/distance.h"

namespace theia {

namespace {

int kNumTrials = 100;

// Zero distance.
TEST(L2Distance, ZeroDistance) {
  Eigen::VectorXf descriptor1(4);
  descriptor1.setZero();
  Eigen::VectorXf descriptor2(4);
  descriptor2.setZero();
  L2 l2_dist;
  ASSERT_EQ(l2_dist(descriptor1, descriptor2), 0);
}

// Known distance.
TEST(L2Distance, KnownDistance) {
  InitRandomGenerator();
  const int num_dimensions = 4;
  Eigen::VectorXf descriptor1(num_dimensions);
  Eigen::VectorXf descriptor2(num_dimensions);
  for (int n = 0; n < kNumTrials; n++) {
    descriptor1.setZero();
    descriptor2.setZero();
    for (int i = 0; i < num_dimensions; i++) {
      float rand_val1 = static_cast<float>(RandDouble(-100, 100));
      float rand_val2 = static_cast<float>(RandDouble(-100, 100));
      descriptor1[i] = rand_val1;
      descriptor2[i] = rand_val2;
    }
    L2 l2_dist;
    ASSERT_DOUBLE_EQ(l2_dist(descriptor1, descriptor2),
                     (descriptor2 - descriptor1).squaredNorm());
  }
}

// Zero distance.
TEST(HammingDistance, ZeroDistance) {
  Eigen::BinaryVectorX descriptor1(512);
  descriptor1.setZero();
  Eigen::BinaryVectorX descriptor2(512);
  descriptor2.setZero();
  Hamming hamming_distance;
  ASSERT_EQ(hamming_distance(descriptor1, descriptor2), 0);
}

// Max distance.
TEST(HammingDistance, MaxDistance) {
  Eigen::BinaryVectorX descriptor1(512);
  descriptor1.setZero();
  Eigen::BinaryVectorX descriptor2(512);
  std::bitset<512>* ones =
      reinterpret_cast<std::bitset<512>*>(descriptor2.data());
  ones->reset().flip();

  Hamming hamming_distance;
  ASSERT_EQ(hamming_distance(descriptor1, descriptor2), 512);
}

// Known distance.
TEST(HammingDistance, KnownDistance) {
  InitRandomGenerator();

  std::bitset<512> zeros;
  zeros.reset();

  Eigen::BinaryVectorX descriptor1(512);
  descriptor1.setZero();

  for (int i = 0; i < kNumTrials; i++) {
    Eigen::BinaryVectorX descriptor2(512);
    descriptor2.setZero();
    std::bitset<512>* rand_bitset =
        reinterpret_cast<std::bitset<512>*>(descriptor2.data());

    int hamming_dist = 0;
    for (int j = 0; j < 512; j++) {
      if (RandDouble(0.0, 1.0) > 0.7) {
        rand_bitset->set(j);
        hamming_dist++;
      }
    }

    Hamming hamming_distance;
    ASSERT_EQ(hamming_distance(descriptor1, descriptor2), hamming_dist);
  }
}

BENCHMARK(HammingDistanceSpeed, SpeedTest, 100, 100) {
  Eigen::BinaryVectorX descriptor1(512);
  Eigen::BinaryVectorX descriptor2(512);
  descriptor1.setZero();
  descriptor2.setRandom();
  Hamming hamming_distance;
  hamming_distance(descriptor1, descriptor2);
}

}  // namespace
}  // namespace theia
