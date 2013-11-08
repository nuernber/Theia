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

#ifndef THEIA_SOLVERS_PROSAC_H_
#define THEIA_SOLVERS_PROSAC_H_

#include <math.h>
#include <cstdlib>
#include <vector>

#include "theia/solvers/estimator.h"
#include "theia/solvers/inlier_support.h"
#include "theia/solvers/prosac_sampler.h"
#include "theia/solvers/sample_consensus_estimator.h"

namespace theia {
// Estimate a model using PROSAC. The Estimate method is inherited, but for
// PROSAC requires the data to be in sorted order by quality (with highest
// quality at index 0).
template <class Datum, class Model>
class Prosac : public SampleConsensusEstimator<Datum, Model> {
 public:
  // min_sample_size: the minimum number of samples needed to estimate a model
  // error_threshold: error threshold for determining if a data point is an
  //   inlier or not.
  // min_num_inliers: Minimum number of inliers needed to terminate.
  // max_iters: Maximum number of iterations to run PROSAC. To set the number
  //   of iterations based on the outlier probability, use SetMaxIters.
  Prosac(int min_sample_size, double error_threshold, int min_num_inliers,
         int max_iters)
      : SampleConsensusEstimator<Datum, Model>(
            new ProsacSampler<Datum>(min_sample_size),
            new InlierSupport(error_threshold, min_num_inliers), max_iters) {}

  // See MaxItersFromOutlierProb for parameters.
  Prosac(int min_sample_size, double error_threshold, int min_num_inliers,
         double outlier_probability, double no_fail_probability)
      : SampleConsensusEstimator<Datum, Model>(
            new ProsacSampler<Datum>(min_sample_size),
            new InlierSupport(error_threshold, min_num_inliers),
            max_iters_(MaxItersFromOutlierProb(min_sample_size,
                                               outlier_probability,
                                               no_fail_probability))) {}
  ~Prosac() {}

 private:
  // Set the max iterations based on Eq. 4.18 in Hartley & Zisserman.
  //   min_sample_size: the min number of samples required to estimate a model.
  //   outlier_probability: prob that a given data point is an outlier
  //   no_fail_probability: prob that at least one sample has no outliers
  //     (typically set to .99)
  int MaxItersFromOutlierProb(int min_sample_size, double outlier_probability,
                              double no_fail_probability = 0.99) {
    return ceil(log(1 - no_fail_probability) /
                log(1.0 - pow(1.0 - outlier_probability, min_sample_size)));
  }
};
}  // namespace theia

#endif  // THEIA_SOLVERS_PROSAC_H_
