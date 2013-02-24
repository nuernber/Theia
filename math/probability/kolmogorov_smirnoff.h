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

#ifndef MATH_PROBABILITY_KOLMOGOROV_SMIRNOFF_TEST_H_
#define MATH_PROBABILITY_KOLMOGOROV_SMIRNOFF_TEST_H_

#include <vector>

namespace math {
namespace probability {
// Two Sample Kolmogorov-Smirnoff Test. This test attempts to determine if two
// residuals come from the same distribution. Residuals passed in must be in
// sorted order. Returns true if the residuals are deemed the same from the
// first n measurements, false if they are not.
bool KolmogorovSmirnoffTest(const std::vector<double>& residual1,
                            const std::vector<double>& residual2,
                            int n1,
                            int n2);

bool KolmogorovSmirnoffTest(const std::vector<double>& residual1,
                            const std::vector<double>& residual2,
                            int n);
// Same as above method, but assumes n is the size of residuals (which must be
// equivalent!).
bool KolmogorovSmirnoffTest(const std::vector<double>& residual1,
                            const std::vector<double>& residual2);

}  // namespace probability
}  // namespace math
#endif  // MATH_PROBABILITY_KOLMOGOROV_SMIRNOFF_TEST_H_
