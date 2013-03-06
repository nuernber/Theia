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
// ARE DISCLAIMED. IN NO EVENT SHALL CHRIS SWEENEY BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#ifndef SOLVERS_MLESAC_H_
#define SOLVERS_MLESAC_H_

#include <vector>

#include "math/distribution.h"
#include "solvers/mle_quality_measurement.h"
#include "solvers/random_sampler.h"
#include "solvers/sample_consensus_estimator.h"

namespace solvers {
template<class Datum, class Model>
class Mlesac : public SampleConsensusEstimator<Datum, Model> {
 public:
  // Params:
  //  min_sample_size: the minimum number of samples needed to estimate a model
  //  inline_mean: Mean of inlier noise distribution.
  //  inlier_sigma: Sigma of the inlier noise distribution.
  //  search_left: Left bound of the search region of image correspondances.
  //    e.g. -100px
  //  search_right: Right bound of the search region of image correspondances.
  //    e.g. 100px
  //  confidence: Vector containing the confidences of each correspondance.
  //  confidence_threshold: Correspondances above this are considered inliers.
  Mlesac(int min_sample_size,
         double inlier_mean,
         double inlier_sigma,
         double search_left,
         double search_right,
         const std::vector<double>& confidence,
         double confidence_threshold)
      : SampleConsensusEstimator<Datum, Model>(
          new RandomSampler<Datum>(min_sample_size),
          new MLEQualityMeasurement(math::NormalDistribution(inlier_mean,
                                                             inlier_sigma),
                                    math::UniformDistribution(search_left,
                                                              search_right),
                                    confidence,
                                    confidence_threshold)) {}

  ~Mlesac() {}
};
}  // namespace solvers
#endif  // SOLVERS_MLESAC_H_
