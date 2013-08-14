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

#include <bitset>
#include "util/util.h"

namespace theia {

template <std::size_t N>
struct Hamming {
  typedef int ResultType;

  ResultType operator()(const Descriptor& a, const Descriptor& b) const {
    // a^b = XOR op, then perform a popcnt on it. This should be optimized
    // rather heavily, and is architecture independent since it is part of the
    // c++ standard. NOTE: cmsweeney tested bitset count vs SSE from BRISK and
    // observed bitset count was roughly ~ 50% faster.
    int dist = 0;
    const int kStep = 128;
    const uchar* a_data = a.CharData();
    const uchar* b_data = b.CharData();
    for (int i = 0; i < a.Dimensions(); i += kStep) {
      const std::bitset<kStep>* binary_a = reinterpret_cast<const std::bitset<kStep>*>(a_data + i);
      const std::bitset<kStep>* binary_b = reinterpret_cast<const std::bitset<kStep>*>(b_data + i);
      dist += (*binary_a ^ *binary_b).count();
    }
    return dist;

    // const std::bitset<N>* binary_a = reinterpret_cast<const std::bitset<N>*>(a.CharData());
    // const std::bitset<N>* binary_b = reinterpret_cast<const std::bitset<N>*>(b.CharData());

    // return (*binary_a ^ *binary_b).count();
  }
};

}  // namespace theia
#endif  // VISION_MATCHING_DISTANCE_HAMMING_H_
