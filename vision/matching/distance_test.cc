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

#include "util/random.h"
#include "vision/matching/distance.h"

namespace theia {
namespace {
int kNumTrials = 100;

void SetFloatValues(const float* float_values, FloatDescriptor* descriptor) {
  for (int i = 0; i < descriptor->Dimensions(); i++) {
    (*descriptor)[i] = float_values[i];
  }
}

template <std::size_t N>
void SetBinaryValues(const std::bitset<N>& bits, BinaryDescriptor* descriptor) {
  std::bitset<N>* binary_descriptor =
      reinterpret_cast<std::bitset<N>*>(descriptor->CharData());
  *binary_descriptor = bits;
}

class TestBinaryDescriptor : public BinaryDescriptor {
 public:
  TestBinaryDescriptor()
      : BinaryDescriptor(const_dimensions_, DescriptorType::OTHER) {}

  int HammingDistance(const BinaryDescriptor& descriptor) const {
    const std::bitset<const_dimensions_>* first_bitset =
        reinterpret_cast<const std::bitset<const_dimensions_>*>(
            this->CharData());
    const std::bitset<const_dimensions_>* second_bitset =
        reinterpret_cast<const std::bitset<const_dimensions_>*>(
            descriptor.CharData());
    return (*first_bitset ^ *second_bitset).count();
  }

 private:
  static constexpr int const_dimensions_ = 512;
};

// Zero distance.
TEST(L2Distance, ZeroDistance) {
  float zeros[4] = { 0.0, 0.0, 0.0, 0.0 };
  FloatDescriptor descriptor1(4, DescriptorType::OTHER);
  SetFloatValues(zeros, &descriptor1);
  FloatDescriptor descriptor2(4, DescriptorType::OTHER);
  SetFloatValues(zeros, &descriptor2);
  L2 l2_dist;
  ASSERT_EQ(l2_dist(descriptor1, descriptor2), 0);
}

// Known distance.
TEST(L2Distance, KnownDistance) {
  InitRandomGenerator();
  float zeros[4] = { 0.0, 0.0, 0.0, 0.0 };
  FloatDescriptor descriptor1(4, DescriptorType::OTHER);
  SetFloatValues(zeros, &descriptor1);
  for (int i = 0; i < kNumTrials; i++) {
    FloatDescriptor descriptor2(4, DescriptorType::OTHER);
    float rand_floats[4];
    float l2_sum = 0.0;
    for (int i = 0; i < 4; i++) {
      float rand_val = RandDouble(-100, 100);
      rand_floats[i] = rand_val;
      l2_sum += rand_val * rand_val;
    }
    SetFloatValues(rand_floats, &descriptor2);
    L2 l2_dist;
    ASSERT_EQ(l2_dist(descriptor1, descriptor2), l2_sum);
  }
}

// Zero distance.
TEST(HammingDistance, ZeroDistance) {
  std::bitset<512> zeros;
  zeros.reset();

  TestBinaryDescriptor descriptor1;
  SetBinaryValues(zeros, &descriptor1);
  TestBinaryDescriptor descriptor2;
  SetBinaryValues(zeros, &descriptor2);
  Hamming hamming_distance;
  ASSERT_EQ(hamming_distance(descriptor1, descriptor2), 0);
}

// Max distance.
TEST(HammingDistance, MaxDistance) {
  std::bitset<512> zeros;
  zeros.reset();
  TestBinaryDescriptor descriptor1;
  SetBinaryValues(zeros, &descriptor1);

  std::bitset<512> ones = zeros.flip();
  TestBinaryDescriptor descriptor2;
  SetBinaryValues(ones, &descriptor2);
  Hamming hamming_distance;
  ASSERT_EQ(hamming_distance(descriptor1, descriptor2), 512);
}

// Known distance.
TEST(HammingDistance, KnownDistance) {
  InitRandomGenerator();

  std::bitset<512> zeros;
  zeros.reset();
  TestBinaryDescriptor descriptor1;
  SetBinaryValues(zeros, &descriptor1);

  for (int i = 0; i < kNumTrials; i++) {
    std::bitset<512> rand_bitset = zeros;
    TestBinaryDescriptor descriptor2;

    int hamming_dist = 0;
    for (int j = 0; j < 512; j++) {
      if (RandDouble(0.0, 1.0) > 0.7) {
        rand_bitset.set(j);
        hamming_dist++;
      }
    }

    SetBinaryValues(rand_bitset, &descriptor2);
    Hamming hamming_distance;
    ASSERT_EQ(hamming_distance(descriptor1, descriptor2), hamming_dist);
  }
}

}  // namespace
}  // namespace theia
