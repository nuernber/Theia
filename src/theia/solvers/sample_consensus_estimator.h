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

#ifndef THEIA_SOLVERS_SAMPLE_CONSENSUS_ESTIMATOR_H_
#define THEIA_SOLVERS_SAMPLE_CONSENSUS_ESTIMATOR_H_

#include <glog/logging.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "theia/solvers/estimator.h"
#include "theia/solvers/quality_measurement.h"
#include "theia/solvers/sampler.h"

namespace theia {

// Helper struct to hold parameters to various RANSAC schemes. error_thresh is
// the threshold for consider data points to be inliers to a model. This is the
// only variable that must be explitly set, and the rest can be used with the
// default values unless other values are desired.
struct RansacParameters {
  RansacParameters()
      : error_thresh(-1),
        failure_probability(0.01),
        min_inlier_ratio(0.1),
        max_iterations(std::numeric_limits<int>::max()),
        use_Tdd_test(false) {}

  // Error threshold to determin inliers for RANSAC (e.g., squared reprojection
  // error). This is what will be used by the estimator to determine inliers.
  double error_thresh;

  // The failure probability of RANSAC. Set to 0.01 means that RANSAC has a 1%
  // chance of missing the correct pose.
  double failure_probability;

  // The minimal assumed inlier ratio, i.e., it is assumed that the given set
  // of correspondences has an inlier ratio of at least min_inlier_ratio.
  // This is required to limit the number of RANSAC iteratios.
  double min_inlier_ratio;

  // Another way to specify the maximal number of RANSAC iterations. In effect,
  // the maximal number of iterations is set to min(max_ransac_iterations, T),
  // where T is the number of iterations corresponding to min_inlier_ratio.
  // This variable is useful if RANSAC is to be applied iteratively, i.e.,
  // first applying RANSAC with an min_inlier_ratio of x, then with one
  // of x-y and so on, and we want to avoid repeating RANSAC iterations.
  // However, the preferable way to limit the number of RANSAC iterations is
  // to set min_inlier_ratio and leave max_ransac_iterations to its default
  // value.
  // Per default, this variable is set to std::numeric_limits<int>::max().
  int max_iterations;

  // Whether to use the T_{d,d}, with d=1, test proposed in
  // Chum, O. and Matas, J.: Randomized RANSAC and T(d,d) test, BMVC 2002.
  // After computing the pose, RANSAC selects one match at random and evaluates
  // all poses. If the point is an outlier to one pose, the corresponding pose
  // is rejected. Notice that if the pose solver returns multiple poses, then
  // at most one pose is correct. If the selected match is correct, then only
  // the correct pose will pass the test. Per default, the test is disabled.
  //
  // NOTE: Not currently implemented!
  bool use_Tdd_test;
};

template <class Datum, class Model> class SampleConsensusEstimator {
 public:
  explicit SampleConsensusEstimator(const int min_num_correspondences)
      : min_sample_size_(min_num_correspondences) {}

  bool Initialize(const RansacParameters& ransac_params);

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

  // Return the number of inliers from the most recent call to Estimate.
  int GetNumInliers() const { return num_inliers_; }

  // Return the number of iterations used from the most recent call to Estimate.
  int GetNumIterations() const { return num_iters_; }

 protected:
  // This method is called from derived classes to set up the sampling scheme
  // and the method for computing inliers. It must be called by derived classes
  // unless they override the Estimate(...) method.
  //
  // RansacParams: the ransac parameters to run the RANSAC scheme with.
  // sampler: The class that instantiates the sampling strategy for this
  //   particular type of sampling consensus.
  // quality_measurement: class that instantiates the quality measurement of
  //   the data. This determines the stopping criterion.
  bool Initialize(const RansacParameters& ransac_params,
                  Sampler<Datum>* sampler,
                  QualityMeasurement* quality_measurement);

  // Computes the maximum number of iterations required to ensure the inlier
  // ratio is the best with a probability corresponding to log_failure_prob.
  int ComputeMaxIterations(const double inlier_ratio,
                           const double log_failure_prob) const;

  // Minimal number of correspondences.
  const int min_sample_size_;

  // Our sampling strategy.
  std::unique_ptr<Sampler<Datum> > sampler_;

  // Our quality metric for the estimated model and data.
  std::unique_ptr<QualityMeasurement> quality_measurement_;

  // Ransac parameters (see above struct).
  RansacParameters ransac_params_;

  // Number of iterations performed before succeeding.
  int num_iters_;

  // Number of inliers from the final result.
  int num_inliers_;
};

// --------------------------- Implementation --------------------------------//

template <class Datum, class Model>
bool SampleConsensusEstimator<Datum, Model>::Initialize(
    const RansacParameters& ransac_params) {
  CHECK_GT(ransac_params.error_thresh, 0)
      << "Error threshold must be set to greater than zero";
  CHECK_LE(ransac_params.min_inlier_ratio, 1.0);
  CHECK_GT(ransac_params.min_inlier_ratio, 0.0);
  CHECK_LT(ransac_params.failure_probability, 1.0);
  CHECK_GT(ransac_params.failure_probability, 0.0);

  ransac_params_ = ransac_params;
  return true;
}

template <class Datum, class Model>
bool SampleConsensusEstimator<Datum, Model>::Initialize(
    const RansacParameters& ransac_params,
    Sampler<Datum>* sampler,
    QualityMeasurement* quality_measurement) {
  CHECK_NOTNULL(sampler);
  CHECK_NOTNULL(quality_measurement);
  sampler_.reset(sampler);
  sampler_->Initialize();

  quality_measurement_.reset(quality_measurement);
  quality_measurement_->Initialize();
  return Initialize(ransac_params);
}

template <class Datum, class Model>
int SampleConsensusEstimator<Datum, Model>::ComputeMaxIterations(
    const double inlier_ratio, const double log_failure_prob) const {
  CHECK_GT(inlier_ratio, 0.0);

  int num_iterations = 1;
  if (inlier_ratio < 1.0) {
    double num_samples = min_sample_size_;
    if (ransac_params_.use_Tdd_test) {
      // If we use the T_{1,1} test, we have to adapt the number of samples
      // that needs to be generated accordingly since we use another
      // match for verification and a correct match is selected with probability
      // inlier_ratio.
      num_samples += 1.0;
    }
    double log_prob = log(1.0 - pow(inlier_ratio, num_samples));
    num_iterations = static_cast<int>(std::floor(
        log_failure_prob / log_prob) + 1.0);
  }
  return std::min(num_iterations, ransac_params_.max_iterations);
}

template <class Datum, class Model>
bool SampleConsensusEstimator<Datum, Model>::Estimate(
    const std::vector<Datum>& data, const Estimator<Datum, Model>& estimator,
    Model* best_model) {
  CHECK_GT(data.size(), 0)
      << "Cannot perform estimation with 0 data measurements!";
  CHECK_NOTNULL(sampler_.get());
  CHECK_NOTNULL(quality_measurement_.get());

  double best_quality = static_cast<double>(QualityMeasurement::INVALID);
  int max_iterations = ransac_params_.max_iterations;
  const double log_failure_prob = log(ransac_params_.failure_probability);

  for (num_iters_ = 0; num_iters_ < max_iterations; num_iters_++) {
    // Sample subset. Proceed if successfully sampled.
    std::vector<Datum> data_subset;
    if (!sampler_->Sample(data, &data_subset)) {
      continue;
    }

    // Estimate model from subset. Skip to next iteration if the model fails to
    // estimate.
    std::vector<Model> temp_models;
    if (!estimator.EstimateModel(data_subset, &temp_models)) {
      continue;
    }

    // Calculate residuals from estimated model.
    for (const Model& temp_model : temp_models) {
      std::vector<double> residuals = estimator.Residuals(data, temp_model);

      // Determine quality of the generated model.
      double sample_quality =
          quality_measurement_->Calculate(residuals);

      // Update best model if error is the best we have seen.
      if (quality_measurement_->Compare(sample_quality, best_quality) ||
          best_quality == static_cast<double>(QualityMeasurement::INVALID)) {
        *best_model = temp_model;
        best_quality = sample_quality;
        max_iterations = ComputeMaxIterations(
            std::max(quality_measurement_->GetInlierRatio(),
                     ransac_params_.min_inlier_ratio),
            log_failure_prob);
      }
    }
  }

  // Grab inliers to refine the model.
  std::vector<bool> temp_inlier_set =
      estimator.GetInliers(data, *best_model, ransac_params_.error_thresh);
  std::vector<Datum> temp_consensus_set;
  for (int i = 0; i < temp_inlier_set.size(); i++) {
    if (temp_inlier_set[i]) {
      temp_consensus_set.push_back(data[i]);
    }
  }

  // Refine the model based on all current inliers.
  estimator.RefineModel(temp_consensus_set, best_model);
  num_inliers_ =
      estimator.GetNumInliers(data, *best_model, ransac_params_.error_thresh);
  return true;
}

}  // namespace theia

#endif  // THEIA_SOLVERS_SAMPLE_CONSENSUS_ESTIMATOR_H_
