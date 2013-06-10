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

#include <time.h>

#include "math/normalized_cross_correlation.h"
#include "gtest/gtest.h"
#include "util/random.h"

TEST(NCC_SSE, check_equal) {
  theia::InitRandomGenerator();
  float avg_speedup = 0.0;
  int num_trials = 0;
  for (int i = 7; i < 70; i++) {
    int arr_size = i*i;
    clock_t sse_t = 0;
    for (int its = 0; its < 1000; its++) {
      float arr1[arr_size];
      float arr2[arr_size];

      // fill arrays.
      for (int j = 0; j < arr_size; j++) {
        arr1[j] = theia::RandDouble(0.0, 1.0);
        arr2[j] = theia::RandDouble(0.0, 1.0);
      }
      clock_t t;
      t = clock();
      float dist2 = theia::NormalizedCrossCorrelation(arr1, arr2, arr_size);
      sse_t += clock() - t;
    }
    VLOG(0) << "[" << i << "x" << i << "] It took "
            << static_cast<float>(sse_t)/CLOCKS_PER_SEC
            << " seconds to do 1000 iterations of sse NCC";
  }
}
