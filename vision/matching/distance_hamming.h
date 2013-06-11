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

#ifndef VISION_MATCHING_DISTANCE_HAMMING_H_
#define VISION_MATCHING_DISTANCE_HAMMING_H_

#ifdef THEIA_USE_SSE
#include <emmintrin.h>
#include <tmmintrin.h>
#endif

#include <bitset>
#include "util/util.h"

namespace theia {
struct Hamming {
  typedef bool ElementType;
  typedef int ResultType;

  template <typename Iterator1, typename Iterator2>
  ResultType operator()(const Iterator1 a, const Iterator2 b,
                        const size_t size) const {
    // a^b = XOR op, then perform a popcnt on it. This should be optimized
    // rather heavily, and is architecture independent since it is part of the
    // c++ standard. NOTE: cmsweeney tested bitset count vs SSE from BRISK and
    // observed bitset count was roughly ~ 50% faster.
    int dist1 = (*a ^ *b).count();
    return dist1;
  }
};

// max_early_dist is the maximum value of the distance of the first num_bits
// bits in order to continue testing the distance.
/*
  template<int max_early_dist = 0, size_t num_early_bits = 128>
  struct HammingEarlyOut {
  typedef bool ElementType;
  typedef int ResultType;

  template <typename Iterator1, typename Iterator2>
  ResultType operator()(Iterator1 a, Iterator2 b, size_t size) const {
  // Test the distance of the first
  std::bitset<size> ones;
  ones.set();
  // Do some bitshifting so that only the first num_early_bits are left as 1's.
  std::bitset<size> first_n_test =
  (ones >> (size - num_early_bits)) << (size - num_early_bits);
  VLOG(0) << "first n: " << fist_n_test.to_string();
  std::bitset<size> rest_test = (ones << num_early_bits) >> num_early_bits;
  VLOG(0) << "rest n: " << rest_test.to_string();
  // Calculate the hamming distance of the first num_early_bits.
  int fist_dist = ((a&first_n_test)^(b&first_n_test)).count();
  if (max_early_dist > 0 && max_early_dist < num_early_bits &&
  fist_dist > max_early_dist) {
  return size_t;
  } else {
  // Otherwise, return the normal distance.
  int rest_dist = ((a&rest_test)^(b&rest_test)).count();
  return fist_dist + rest_dist;
  // return (a^b).count();
  }
  }
  };
*/
}  // namespace theia
#endif  // VISION_MATCHING_DISTANCE_HAMMING_H_
