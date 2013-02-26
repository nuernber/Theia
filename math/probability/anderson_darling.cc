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
namespace {
// TODO(cmsweeney): This is surely a useful function. We should incorporate it
// into its ownpublicly accesible class/method at a later date.
// Calculates the CDF of a normal distribution (i.e. the phi function).
double Phi(double x) {
    // constants
    double a1 =  0.254829592;
    double a2 = -0.284496736;
    double a3 =  1.421413741;
    double a4 = -1.453152027;
    double a5 =  1.061405429;
    double p  =  0.3275911;

    // Save the sign of x
    int sign = 1;
    if (x < 0)
        sign = -1;
    x = fabs(x)/sqrt(2.0);

    // A&S formula 7.1.26
    double t = 1.0/(1.0 + p*x);
    double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

    return 0.5*(1.0 + sign*y);
}
}  // namespace

bool AndersonDarlingTest(const std::vector<double>& residual,
                         int n) {
  CHECK_LE(n, residual.size()) << "n must be a valid size";
  CHECK_GE(n, 0) << "n must be a valid size";

  // Calculate mean.
  double mean = 0;
  for (int i = 0; i < n; i++)
    mean += residual[i];
  mean /= n;
  VLOG(0) << "mean = " << mean;
  // Calculate variance.
  double st_dev = 0;
  for (int i = 0; i < n; i++)
    st_dev += (residual[i] - mean)*(residual[i] - mean);
  st_dev /= n - 1;
  st_dev = sqrt(st_dev);

  VLOG(0) << "st dev = " << st_dev;
  
  // Calculate standardized data points.
  std::vector<double> norm_residuals(n);
  for (int i = 0; i < n; i++)
    norm_residuals[i] = (residual[i] - mean)/st_dev;

  // Calculate A2, the A.D. statistic.
  double a2 = 0;
  for (int i = 0; i < n; i++) {
    a2 += (2.0*i + 1.0)*log(Phi(norm_residuals[i])) +
        (2.0*(n - 1.0 - i) + 1.0)*log(1.0 - Phi(norm_residuals[i]));
  }
  a2 = -n - a2/n;
  VLOG(0) << "a2 = " << a2;
  
  double a_star = a2*(1.0 + 0.75/n + 2.25/(n*n));
  VLOG(0) << "a_star = " << a_star;

  // 5% rejection rate of null hypothesis.
  return a_star < 0.752;  
}

bool AndersonDarlingTest(const std::vector<double>& residual) {
  return AndersonDarlingTest(residual, residual.size());
}

}  // namespace probability
}  // namespace math
