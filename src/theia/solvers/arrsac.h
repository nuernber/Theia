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

#ifndef THEIA_SOLVERS_ARRSAC_H_
#define THEIA_SOLVERS_ARRSAC_H_

#include <chrono>
#include <random>

#include <algorithm>
#include <memory>
#include <vector>

#include "theia/math/probability/sequential_probability_ratio.h"
#include "theia/solvers/estimator.h"
#include "theia/solvers/inlier_support.h"
#include "theia/solvers/sample_consensus_estimator.h"
#include "theia/solvers/random_sampler.h"
#include "theia/solvers/prosac_sampler.h"

// Implementation of ARRSAC, a "real-time" RANSAC algorithm, by Raguram
// et. al. (ECCV 2008). You only need to call the constructor and the Compute
// method to run ARRSAC on your data.
namespace theia {
// Helper struct for scoring the hypotheses in ARRSAC.
template <class Datum> struct ScoredData {
  Datum data;
  double score;
  ScoredData() {}
  ScoredData(const Datum& _data, double _score) : data(_data), score(_score) {}
};

// Comparator method so that we can call c++ algorithm sort.
template <class Datum>
bool CompareScoredData(ScoredData<Datum> i, ScoredData<Datum> j) {
  return i.score < j.score;
}

template <class Datum, class Model>
class Arrsac : public SampleConsensusEstimator<Datum, Model> {
 public:
  // Params:
  //   min_sample_size: The minimum number of samples needed to estimate a
  //     model.
  //   error_thresh: Error threshold for determining inliers vs. outliers. i.e.
  //     if the error is below this, the data point is an inlier.
  //   max_candidate_hyps: Maximum number of hypotheses in the initial
  //     hypothesis set
  //   block_size: Number of data points a hypothesis is evaluated against
  //     before preemptive ordering is used.
  Arrsac(int min_sample_size, double error_thresh, int max_candidate_hyps = 500,
         int block_size = 100)
      : min_sample_size_(min_sample_size),
        error_thresh_(error_thresh),
        max_candidate_hyps_(max_candidate_hyps),
        block_size_(block_size),
        sigma_(0.05),
        epsilon_(0.1),
        inlier_confidence_(0.95) {}
  ~Arrsac() {}

  // Set sigma and epsilon SPRT params (see sequential_probability_ratio_test.h
  // for more information). This is an optional method, as it is only an initial
  // estimation of these parameters -- sigma and epsilon are adjusted and
  // re-estimated as the ARRSAC algorithm progresses.
  // inlier_confidence: confidence that there exists one set with no outliers.
  void SetOptionalParameters(double sigma, double epsilon,
                             double inlier_confidence) {
    sigma_ = sigma;
    epsilon_ = epsilon;
    inlier_confidence_ = inlier_confidence;
  }

  // Algorithm 2 in Raguram et. al.'s ARRSAC paper.
  // Params:
  //   data: Input data to generate a model from.
  //   estimator: Derived class used to esimate the model.
  //   best_model: Output parameter that will be filled with the best estimated
  //     model on success.
  // Return: true on successful estimation, false otherwise.
  bool Estimate(const std::vector<Datum>& data,
                const Estimator<Datum, Model>& estimator, Model* best_model);

  // This is sort of a hack. We make this method protected so that we can test
  // it easily. See arrsac_test.cc for more.
 protected:
  // Algorithm 3 in Raguram et. al.'s ARRSAC paper. Given data, generate an
  // initial set of hypotheses from a PROSAC-style sampling. This initial set of
  // hypotheses will be used to generate more hypotheses in the Compute
  // method. Returns the set of initial hypotheses.
  int GenerateInitialHypothesisSet(const std::vector<Datum>& data_input,
                                   const Estimator<Datum, Model>& estimator,
                                   std::vector<Model>* accepted_hypotheses);

 private:
  // Minimum sample size to generate a model.
  int min_sample_size_;

  // Threshold for determining inliers.
  double error_thresh_;

  // Maximum candidate hypotheses to consider at any time.
  int max_candidate_hyps_;

  // The number of data points that the hypothesis is evaluated against before
  // preemption and re-ordering takes place.
  int block_size_;

  // SPRT Parameters. These parameters are tuned and updated as ARRSAC iterates.
  // Type 1 error estimation.
  double sigma_;

  // Estimated inlier ratio.
  double epsilon_;

  // Confidence that there exists at least one set with no outliers.
  double inlier_confidence_;

  // Inliners from the recent data. Only valid if Estimate has been called!
  std::vector<bool> inliers_;

};

// -------------------------- Implementation -------------------------- //

template <class Datum, class Model>
int Arrsac<Datum, Model>::GenerateInitialHypothesisSet(
    const std::vector<Datum>& data_input,
    const Estimator<Datum, Model>& estimator,
    std::vector<Model>* accepted_hypotheses) {
  //   set parameters for SPRT test, calculate initial value of A
  double decision_threshold = CalculateSPRTDecisionThreshold(sigma_, epsilon_);
  int k = 1;
  int m_prime = max_candidate_hyps_;
  // Inner RANSAC variables.
  int inner_ransac_its = 0;
  bool inner_ransac = false;
  int max_inner_ransac_its = 10000;
  int max_num_inliers = 0;

  // We need a local copy of the data input so that we can modify/resize it for
  // inner ransac (which uses inliers from previous results as the sampling
  // universe).
  std::vector<Datum> data;

  // Vars to keep track of the avg inlier ratio of rejected hypotheses.
  int num_rejected_hypotheses = 0;
  double rejected_accum_inlier_ratio = 0;

  // RandomSampler and PROSAC Sampler.
  RandomSampler<Datum> random_sampler(min_sample_size_);
  ProsacSampler<Datum> prosac_sampler(min_sample_size_);
  while (k <= m_prime) {
    std::vector<Model> hypotheses;
    if (!inner_ransac) {
      // Generate hypothesis h(k) with k-th PROSAC sample.
      std::vector<Datum> prosac_subset;
      prosac_sampler.SetSampleNumber(k);
      prosac_sampler.Sample(data_input, &prosac_subset);
      estimator.EstimateModel(prosac_subset, &hypotheses);
    } else {
      // Generate hypothesis h(k) with subset generated from inliers of a
      // previous hypothesis.
      std::vector<Datum> random_subset;
      random_sampler.Sample(data, &random_subset);
      estimator.EstimateModel(random_subset, &hypotheses);

      inner_ransac_its++;
      if (inner_ransac_its == max_inner_ransac_its) {
        inner_ransac_its = 0;
        inner_ransac = false;
      }
    }

    for (const Model& hypothesis : hypotheses) {
      int num_tested_points;
      double observed_inlier_ratio;
      // Evaluate hypothesis h(k) with SPRT.
      std::vector<double> residuals =
          estimator.Residuals(data_input, hypothesis);
      bool sprt_test = SequentialProbabilityRatioTest(
          residuals, error_thresh_, sigma_, epsilon_, decision_threshold,
          &num_tested_points, &observed_inlier_ratio);

      // If the model was rejected by the SPRT test.
      if (!sprt_test) {
        // re-estimate params of SPRT (if required)
        // sigma = average of inlier ratios in bad models
        // TODO(cmsweeney): determine if this estimation (and epsilon) is:
        //    number of inliers observed / total number of points    or
        //    number of inliers observed / number of points observed in SPRT
        rejected_accum_inlier_ratio += observed_inlier_ratio;
        num_rejected_hypotheses++;
        sigma_ = rejected_accum_inlier_ratio /
                 static_cast<double>(num_rejected_hypotheses);
      } else if (floor(observed_inlier_ratio * num_tested_points) >
                 max_num_inliers) {
        // Else if hypothesis h(k) is accepted and has the largest support so
        // far.
        accepted_hypotheses->push_back(hypothesis);
        max_num_inliers = floor(observed_inlier_ratio * num_tested_points);

        // Set parameters to force inner ransac to execute.
        inner_ransac = true;
        inner_ransac_its = 0;

        // Set U_in = support of hypothesis h(k).
        data.clear();
        for (int i = 0; i < data_input.size(); i++) {
          if (estimator.Error(data_input[i], hypothesis) < error_thresh_)
            data.push_back(data_input[i]);
        }

        // Re-estimate params of SPRT.
        // Estimate epsilon as inlier ratio for largest size of support.
        epsilon_ = static_cast<double>(max_num_inliers) /
                   static_cast<double>(data_input.size());
        // estimate inlier ratio e' and M_prime (eq 1) Cap M_prime at max of M
        // TODO(cmsweeney): verify that num_tested_points is the correct value
        // here and not data_input.size().
        int m_prime =
            ceil(log(1 - inlier_confidence_) /
                 log(1 - pow(observed_inlier_ratio, num_tested_points)));
        m_prime = std::max(max_candidate_hyps_, m_prime);
      }
    }
    k++;
  }
  return k;
}

template <class Datum, class Model>
bool Arrsac<Datum, Model>::Estimate(const std::vector<Datum>& data,
                                    const Estimator<Datum, Model>& estimator,
                                    Model* best_model) {
  // Generate Initial Hypothesis Test
  std::vector<Model> initial_hypotheses;
  int k = GenerateInitialHypothesisSet(data, estimator, &initial_hypotheses);

  // Score initial set.
  std::vector<ScoredData<Model> > hypotheses(initial_hypotheses.size());
  for (int i = 0; i < hypotheses.size(); i++) {
    hypotheses[i] = ScoredData<Model>(initial_hypotheses[i], 0.0);
    // Calculate inlier score for the hypothesis.
    for (int j = 0; j <= block_size_; j++) {
      if (estimator.Error(data[j], hypotheses[i].data) < error_thresh_)
        hypotheses[i].score += 1.0;
    }
  }

  RandomSampler<Datum> random_sampler(min_sample_size_);

  // Preemptive Evaluation
  for (int i = block_size_ + 1; i < data.size(); i++) {
    // Select n, the number of hypotheses to consider.
    int f_i =
        floor(max_candidate_hyps_ * pow(2, -1.0 * floor(i / block_size_)));
    int n = std::min(f_i, static_cast<int>(hypotheses.size() / 2));

    // Rorder and select hypothesis h(1)...h(n). Should be really fast because
    // data point can only increase by 1 or not change.
    sort(hypotheses.begin(), hypotheses.end(), CompareScoredData<Model>);
    hypotheses.resize(n);

    if (n == 1) {
      break;
    }

    // Score the hypotheses using data point i.
    for (int j = 0; j < hypotheses.size(); j++) {
      if (estimator.Error(data[i], hypotheses[j].data) < error_thresh_)
        hypotheses[i].score += 1.0;
    }

    if (i % block_size_ == 0) {
      // Calculate best inlier ratio e' and num hypotheses M' (eq. 1).
      // Use a simple for loop. This should be really fast since the list was
      // recently sorted and the values can only have increased by 1.
      double max_inliers = 0;
      for (int j = 0; j < hypotheses.size() - 1; j++) {
        if (hypotheses[j].score > hypotheses[j + 1].score) {
          max_inliers = hypotheses[j].score;
          break;
        }
      }

      // Estimate best inlier ratio.
      double inlier_ratio_estimate = max_inliers / static_cast<double>(i);
      // Calculate number of hypotheses needed (eq. 1).
      int temp_max_candidate_hyps =
          static_cast<int>(ceil(log(1 - inlier_confidence_) /
                                log(1 - pow(inlier_ratio_estimate, i))));
      // M' = max(M,M').
      temp_max_candidate_hyps =
          std::max(max_candidate_hyps_, temp_max_candidate_hyps);

      // If we need more hypotheses, generate them now.
      if (temp_max_candidate_hyps > k) {
        // Generate and evaluate M' - k new hypotheses on i data points.
        for (int j = 0; j < temp_max_candidate_hyps - k; j++) {
          std::vector<Datum> data_random_subset;
          random_sampler.Sample(data, &data_random_subset);

          // Estimate new hypothesis model.
          std::vector<Model> estimated_models;
          estimator.EstimateModel(data_random_subset, &estimated_models);
          for (const Model& estimated_model : estimated_models) {
            ScoredData<Model> new_hypothesis(estimated_model, 0.0);
            // Score the newly generated model.
            for (int l = 0; l < i; l++)
              if (estimator.Error(data[l], new_hypothesis.data) < error_thresh_)
                new_hypothesis.score += 1.0;
            // Add newly generated model to the hypothesis set.
            hypotheses.push_back(new_hypothesis);
          }
        }

        // Update k to be our new maximum candidate hypothesis size.
        k = temp_max_candidate_hyps;
      }
    }
  }

  if (hypotheses.size() > 0) {
    // The best model should be at the beginning of the list since we only quit
    // when n==1.
    *best_model = hypotheses[0].data;
  } else {
    return false;
  }

  // Grab inliers to refine the model.
  InlierSupport quality_measurement(error_thresh_, 0);
  std::vector<double> residuals = estimator.Residuals(data, *best_model);
  std::vector<bool> temp_inlier_set(data.size());
  quality_measurement.Calculate(residuals, &temp_inlier_set);
  std::vector<Datum> temp_consensus_set;
  for (int i = 0; i < temp_inlier_set.size(); i++) {
    if (temp_inlier_set[i]) {
      temp_consensus_set.push_back(data[i]);
    }
  }
  // Refine the model based on all current inliers.
  estimator.RefineModel(temp_consensus_set, best_model);

  // Calculate the final inliers.
  inliers_.resize(data.size());
  std::vector<double> final_residuals = estimator.Residuals(data, *best_model);
  quality_measurement.Calculate(final_residuals, &inliers_);

  return true;
}

}  // namespace theia

#endif  // THEIA_SOLVERS_ARRSAC_H_
