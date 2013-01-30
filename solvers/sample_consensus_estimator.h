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

#ifndef SOLVERS_SAMPLE_CONSENSUS_ESTIMATOR_H_
#define SOLVERS_SAMPLE_CONSENSUS_ESTIMATOR_H_

#include <glog/logging.h>
#include <memory>
#include <vector>

#include "solvers/estimator.h"
#include "solvers/quality_measurement.h"
#include "solvers/sampler.h"

namespace solvers {
template<class Datum, class Model>
class SampleConsensusEstimator {
 public:
  // sampler: The class that instantiates the sampling strategy for this
  //   particular type of sampling consensus.
  // quality_measurement: class that instantiates the quality measurement of
  //   the data. This determines the stopping criterion.
  // max_iters: Maximum number of iterations to run RANSAC. To set the number
  //   of iterations based on the outlier probability, use SetMaxIters.
  SampleConsensusEstimator(Sampler<Datum>* sampler,
                           QualityMeasurement* quality_measurement,
                           int max_iters = 10000)
      : max_iters_(max_iters),
        num_iters_(-1) {
    sampler_.reset(sampler);
    quality_measurement_.reset(quality_measurement);
  }

  virtual ~SampleConsensusEstimator() {}

  // Computes the best-fitting model using RANSAC. Returns false if RANSAC
  // calculation fails and true (with the best_model output) if successful.
  // Params:
  //   data: the set from which to sample
  //   estimator: The estimator used to estimate the model based on the Datum
  //     and Model type
  //   best_model: The output parameter that will be filled with the best model
  //     estimated from RANSAC
  virtual bool Estimate(const std::vector<Datum>& data,
                        const Estimator<Datum, Model>& estimator,
                        Model* best_model);

  // Returns a bool vector with true for inliers, false for outliers.
  const std::vector<bool>& GetInliers() { return inliers_; }

  // Count the number of inliers.
  int GetNumInliers() {
    int num_inliers = 0;
    for (bool inlier : inliers_) {
      if (inlier)
        num_inliers++;
    }
    return num_inliers;
  }

  int GetNumIterations() {
    return num_iters_;
  }
  
 protected:
  // Our sampling strategy.
  std::unique_ptr<Sampler<Datum> > sampler_;

  // Our quality metric for the estimated model and data.
  std::unique_ptr<QualityMeasurement> quality_measurement_;

  // Inliners from the recent data. Only valid if Estimate has been called!
  std::vector<bool> inliers_;

  // Max number of iterations to perform before terminating .
  int max_iters_;

  // Number of iterations performed before succeeding.
  int num_iters_;
};

template<class Datum, class Model>
bool SampleConsensusEstimator<Datum, Model>::Estimate(
    const std::vector<Datum>& data,
    const Estimator<Datum, Model>& estimator,
    Model* best_model) {
  CHECK_GT(data.size(), 0)
      << "Cannot perform estimation with 0 data measurements!";

  double best_quality = static_cast<double>(QualityMeasurement::INVALID);
  for (int iters = 0; iters < max_iters_; iters++) {
    // Sample subset. Proceed if successfully sampled.
    std::vector<Datum> data_subset;
    if (!sampler_->Sample(data, &data_subset))
      continue;

    // Estimate model from subset. Skip to next iteration if the model fails to
    // estimate.
    Model temp_model;
    if (!estimator.EstimateModel(data_subset, &temp_model))
      continue;

    // Calculate residuals from estimated model.
    std::vector<double> residuals(data.size());
    for (int i = 0; i < data.size(); i++)
      residuals[i] = estimator.Error(data[i], temp_model);

    // Determine quality of the generated model.
    std::vector<bool> temp_inlier_set(data.size());
    double sample_quality = quality_measurement_->Calculate(residuals,
                                                            &temp_inlier_set);

    // Update best model if error is the best we have seen.
    if (quality_measurement_->Compare(sample_quality, best_quality) ||
        best_quality == static_cast<double>(QualityMeasurement::INVALID)) {
      *best_model = temp_model;
      best_quality = sample_quality;

      // If the inlier termination criterion is met, re-estimate the model
      // based on all inliers and return;
      if (quality_measurement_->SufficientlyHighQuality(best_quality)) {
        // Grab inliers to refine the model.
        std::vector<Datum> temp_consensus_set;
        for (int i = 0; i < temp_inlier_set.size(); i++) {
          if (temp_inlier_set[i])
            temp_consensus_set.push_back(data[i]);
        }
        // Refine the model based on all current inliers.
        estimator.RefineModel(temp_consensus_set, best_model);
        num_iters_ = iters + 1;
        break;
      }
    }
  }

  inliers_.resize(data.size());
  std::vector<double> final_residuals(data.size());
  for (int i = 0; i < data.size(); i++)
    final_residuals[i] = estimator.Error(data[i], *best_model);

  quality_measurement_->Calculate(final_residuals, &inliers_);

  return true;
}
}  // namespace solvers

#endif  // SOLVERS_SAMPLE_CONSENSUS_ESTIMATOR_H_
