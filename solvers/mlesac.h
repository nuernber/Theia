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

#ifndef SOLVERS_MLESAC_H_
#define SOLVERS_MLESAC_H_

#include <vector>

#include "solvers/mle_quality_measurement.h"
#include "solvers/random_sampler.h"
#include "solvers/sample_consensus_estimator.h"

namespace solvers {


template<class Datum, class Model>
class Mlesac : public SampleConsensusEstimator<Datum, Model> {
 public:
  // min_sample_size: the minimum number of samples needed to estimate a model
  // error_threshold: error threshold for determining if a data point is an
  //   inlier or not.
  // min_num_inliers: Minimum number of inliers needed to terminate.
  // max_iters: Maximum number of iterations to run RANSAC. To set the number
  //   of iterations based on the outlier probability, use SetMaxIters.
  Mlesac(int min_sample_size,
         double error_threshold,
         int min_num_inliers,
         int max_iters)
      : SampleConsensusEstimator<Datum, Model>(
          new RandomSampler<Datum>(min_sample_size),
          new MLEQualityMeasurement(error_threshold,
                                    min_num_inliers),
          max_iters) {}

  ~Ransac() {}

 private:
};


}  // namespace solvers
#endif  // SOLVERS_MLESAC_H_
