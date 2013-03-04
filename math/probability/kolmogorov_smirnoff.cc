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

#include "math/probability/kolmogorov_smirnoff.h"

#include <algorithm>
#include <cmath>
#include <vector>
#include <glog/logging.h>
#include <iostream>
#include <fstream>

namespace math {
namespace probability {
bool KolmogorovSmirnoffTest(const std::vector<double>& residual1,
                            const std::vector<double>& residual2,
                            int n1,
                            int n2) {
  CHECK(n1 <= residual1.size() && n2 <= residual2.size())
      << "n is too large in KS Test!";

  int r1_index = 0;
  int r2_index = 0;
  double max_distance = 0;

  double rdiff = 0;
  double diff1 = 1.0/n1;
  double diff2 = 1.0/n2;
  for (int i = 0; i < n1 + n2; i++) {
    if (residual1[r1_index] < residual2[r2_index]) {
      rdiff -= diff1;
      r1_index++;
    } else if (residual1[r1_index] > residual2[r2_index]) {
      rdiff += diff2;
      r2_index++;
    } else {
      double x = residual1[r1_index];
      while (residual1[r1_index] == x && r1_index < n1) {
        rdiff -= diff1;
        r1_index++;
      }
      while (residual2[r2_index] == x && r2_index < n2) {
        rdiff += diff2;
        r2_index++;
      }
    }
    max_distance = std::max(max_distance, fabs(rdiff));
    if (r1_index >= n1 || r2_index >= n2)
      break;
  }
  
  // 1.36 is the value in the published tables for 5% rejection rate of the null
  // hypothesis.
  double val =
      sqrt(static_cast<double>(n1*n2)/static_cast<double>(n1+n2))*max_distance;
  
  return val <= 1.36;
}


bool KolmogorovSmirnoffTest(const std::vector<double>& residual1,
                            const std::vector<double>& residual2,
                            int n) {
  return KolmogorovSmirnoffTest(residual1, residual2, n, n);
}

bool KolmogorovSmirnoffTest(const std::vector<double>& residual1,
                            const std::vector<double>& residual2) {
  int min_residual_size = (residual1.size() < residual2.size()) ?
      residual1.size() : residual2.size();
  return KolmogorovSmirnoffTest(residual1, residual2, min_residual_size);
}

}  // namespace probability
}  // namespace math
