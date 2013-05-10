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

#ifndef SOLVERS_RECON_H_
#define SOLVERS_RECON_H_

#include <math.h>
#include <unordered_map>

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <vector>

#include "math/probability/kolmogorov_smirnoff.h"
#include "solvers/estimator.h"
#include "solvers/inlier_support.h"
#include "solvers/prosac_sampler.h"
#include "solvers/random_sampler.h"
#include "solvers/sample_consensus_estimator.h"

namespace theia {
// Implementation of the RECON method by Raguram et al: "RECON: Scale-Adaptive
// Robust Estimation via Residual Consensus". This RANSAC variation determines
// the inlier threhold automatically by generating several models, then
// comparing the distribution of the residuals of those models. For good models,
// the distributions of residuals should demonstrate a clear compatibility
// separating the inlier distribution and the outlier distribution, so the
// inlier ratio can be automatically determined in this way.
template<class Datum, class Model>
class Recon : public SampleConsensusEstimator<Datum, Model> {
 public:
  // min_sample_size: the minimum number of samples needed to estimate a model
  // sigma_max: A *rough* estimate of the max noise variance. This can be up to
  //   an order of magnitude and still be useful.
  // min_consistent_models: Number of consistent models needed before a solution
  //   is determined (typically this is set to 3).
  Recon(int min_sample_size,
        int min_consistent_models,
        double sig_max)
      : min_sample_size_(min_sample_size),
        min_consistent_models_(min_consistent_models),
        sigma_max(sig_max),
        alpha(0.99),
        num_nonminimal_models(20) {}

  ~Recon() {}

  // Computes the best-fitting model using RECON. Returns false if RECON
  // calculation fails and true (with the best_model output) if
  // successful. Implementation of Algorithm 2 in the RECON paper.
  // Params:
  //   data: the set from which to sample
  //   estimator: The estimator used to estimate the model based on the Datum
  //     and Model type
  //   best_model: The output parameter that will be filled with the best model
  //     estimated from RANSAC
  bool Estimate(const std::vector<Datum>& data,
                const Estimator<Datum, Model>& estimator,
                Model* best_model);

  // % of common inliers = alpha*alpha
  double alpha;

  // max sigma as approximated by guess of outlier ratio
  double sigma_max;

  // Number of nonminimal models to generate using PROSAC sampling on mean of
  // best residuals (Step 4).
  int num_nonminimal_models;

 private:
  // Minimum number of samples needed to generate a model.
  int min_sample_size_;

  // Minimum number of models that must be consistent before determining
  // convergence.
  int min_consistent_models_;

  class ResidualDataPoint {
   public:
    ResidualDataPoint() : index_(-1), residual_(-1) {}
    ResidualDataPoint(int index, double residual)
        : index_(index), residual_(residual) {}
    int GetIndex() const { return index_; }
    double GetResidual() const { return residual_; }
    void SetResidual(double resid) { residual_ = resid; }
    static bool SortByIndex(const ResidualDataPoint& first,
                            const ResidualDataPoint& second) {
      return first.GetIndex() < second.GetIndex();
    }
    static bool SortByNormResidual(const ResidualDataPoint& first,
                                   const ResidualDataPoint& second) {
      return first.GetResidual()*first.GetResidual() <
        second.GetResidual()*second.GetResidual();
    }

   private:
    int index_;
    double residual_;
  };
  typedef std::vector<ResidualDataPoint> ModelResiduals;

  // Finds alpha consistent models for hte data set. Outputs the alpha
  // consistent models and the indices of their overlap.
  void FindAlphaConsistentModels(
      const std::vector<Datum>& data,
      const Estimator<Datum, Model>& estimator,
      std::vector<ModelResiduals>* alpha_consistent_models,
      std::vector<int>* inliers);

  // Given a set of models, computes the mean of the residuals and returns the
  // corresponding data points in sorted order.
  void ComputeMeanInliers(const std::vector<Datum>& data,
                          const std::vector<int>& inliers,
                          std::vector<ModelResiduals>* models,
                          std::vector<Datum>* mean_inliers);

  // Generates prosac models from the data supplied.
  void GenerateProsacModels(const std::vector<Datum>& data,
                            const Estimator<Datum, Model>& estimator,
                            int num_models,
                            std::vector<ModelResiduals>* models);

  // Finds the best inlier set based on the data and models given.
  void FindBestInlierSet(const std::vector<Datum>& data,
                         const std::vector<ModelResiduals>& models,
                         std::vector<Datum>* best_data);

  // Test if two models are alpha consistent i.e. have > alpha^2 % of common
  // inliers in the first n data points (sorted by residual). n and the common
  // indices in the first n points are output, and the result of test is
  // returned.
  bool AlphaConsistent(const ModelResiduals& m1,
                       const ModelResiduals& m2,
                       int* n,
                       std::vector<int>* common_ind) const;

  // Wrapper for the KS test within the probability moduele. Determines whether
  // the two distributions r1 and r2 come from the same underlying
  // distribtuion. Only the data points found in common_indices are passed to
  // the KS test (as these are the estimated inliers).
  // NOTE: I have not found this test to be very robust for line-fitting.
  // TODO(cmsweeney): Test this for more advanced model-fitting (i.e.
  // homography).
  bool KolmogorovSmirnoffWrapper(const ModelResiduals& r1,
                                 const ModelResiduals& r2,
                                 const std::vector<int>& common_indices) const;

  // Utility function to intersct set_1 with set_1 and store the values in
  // set_1.
  void IntersectInPlace(std::vector<int>* set_1,
                        const std::vector<int>& set_2);
};

template<class Datum, class Model>
bool Recon<Datum, Model>::AlphaConsistent(const ModelResiduals& m1,
                                          const ModelResiduals& m2,
                                          int* n,
                                          std::vector<int>* common_ind) const {
  CHECK_EQ(m1.size(), m2.size()) << "data must be of same size!";
  double alpha_sq = alpha*alpha;
  std::vector<bool> overlap(m1.size(), false);
  int num_overlap = 0;
  // Loop over the data, setting values in the overlap to true when a data
  // point is visited. If we reach a data point that has already been visited,
  // then this point is part of the overlap.
  for (int i = 0; i < 0.9*m1.size(); i++) {
    // If the indices are equal, or if they have previously been visited then
    // we have a common member.
    // Check if m1's next point has been visited.
    if (overlap[m1[i].GetIndex()]) {
      num_overlap++;
      common_ind->push_back(m1[i].GetIndex());
    }
    // Check if m2's next point has been visited.
    if (overlap[m2[i].GetIndex()]) {
      num_overlap++;
      common_ind->push_back(m2[i].GetIndex());
    }

    // Check if both data points are the same index.
    if (m1[i].GetIndex() == m2[i].GetIndex()) {
      num_overlap++;
      common_ind->push_back(m1[i].GetIndex());
    }

    overlap[m1[i].GetIndex()] = true;
    overlap[m2[i].GetIndex()] = true;

    if (num_overlap/static_cast<double>(i + 1.0) >=
        alpha_sq && i > m1.size()/10) {
      *n = i + 1;
      return true;
    }
  }
  return false;
}

template<class Datum, class Model>
bool Recon<Datum, Model>::KolmogorovSmirnoffWrapper(
    const ModelResiduals& r1,
    const ModelResiduals& r2,
    const std::vector<int>& common_indices) const {
  std::vector<double> residuals1;
  residuals1.reserve(common_indices.size());
  std::vector<double> residuals2;
  residuals2.reserve(common_indices.size());
  for (int common_index : common_indices) {
    residuals1.push_back(r1[common_index].GetResidual());
    residuals2.push_back(r2[common_index].GetResidual());
  }
  std::sort(residuals1.begin(), residuals1.end());
  std::sort(residuals2.begin(), residuals2.end());

  return KolmogorovSmirnoffTest(residuals1, residuals2);
}

template<class Datum, class Model>
void Recon<Datum, Model>::IntersectInPlace(std::vector<int>* set_1,
                                           const std::vector<int>& set_2) {
  std::vector<int>::iterator it1 = set_1->begin();
  std::vector<int>::const_iterator it2 = set_2.begin();
  while ( (it1 != set_1->end()) && (it2 != set_2.end()) ) {
    if (*it1 < *it2) {
      it1 = set_1->erase(it1);
    } else if (*it2 < *it1) {
      ++it2;
    } else {
      ++it1;
      ++it2;
    }
  }
  // Anything left in set_1 from here on did not appear in set_2,
  // so we remove it.
  set_1->erase(it1, set_1->end());
}

template<class Datum, class Model>
void Recon<Datum, Model>::FindAlphaConsistentModels(
    const std::vector<Datum>& data,
    const Estimator<Datum, Model>& estimator,
    std::vector<ModelResiduals>* alpha_consistent_models,
    std::vector<int>* inliers) {
  RandomSampler<Datum> random_sampler(min_sample_size_);
  std::vector<ModelResiduals> valid_models;
  std::vector<Model> all_models;
  alpha_consistent_models->clear();
  // Repeat until we have the minimum number of consistent models needed to
  // determine a convergence.
  std::vector<std::vector<int> > potential_inliers;
  while (alpha_consistent_models->size() < min_consistent_models_) {
    // Estimate a model from a random sample.
    std::vector<Datum> data_subset;
    if (!random_sampler.Sample(data, &data_subset))
      continue;
    Model temp_model;
    if (!estimator.EstimateModel(data_subset, &temp_model))
      continue;

    // Early exit if the model is not valid.
    if (!estimator.ValidModel(temp_model))
      continue;
    all_models.push_back(temp_model);

    // Get residuals
    std::vector<double> current_residuals = estimator.Residuals(data,
                                                                temp_model);

    // Sort data by residuals.
    ModelResiduals current_model;
    current_model.reserve(current_residuals.size());
    for (int i = 0; i < current_residuals.size(); i++)
      current_model.push_back(ResidualDataPoint(i, current_residuals[i]));
    std::sort(current_model.begin(),
              current_model.end(),
              ResidualDataPoint::SortByNormResidual);
    valid_models.push_back(current_model);

    potential_inliers.clear();
    alpha_consistent_models->clear();
    alpha_consistent_models->push_back(current_model);
    VLOG(0) << "for model = " << temp_model.m << ", " << temp_model.b;
    // Loop over all previous models and test for alpha consistency. Exit early
    // if we find enough alpha consistent models.
    std::vector<int> overlap_count;
    for (int i = 0;
         i < valid_models.size() - 1 &&
             alpha_consistent_models->size() < min_consistent_models_;
         i++) {
      int n;
      std::vector<int> common_indices;
      // Test for alpha consistency
      if (AlphaConsistent(current_model,
                          valid_models[i],
                          &n,
                          &common_indices)) {
        VLOG(0) << "\tmodel " << all_models[i].m << ", "
                << all_models[i].b << " is valid at " << n;

        double sigma_hat = 1.4286*(1.0 + 5.0/(n - min_sample_size_))*
            fabs(current_model[n/2].GetResidual());
        // Enusre the that size of overlap sets are relatively close.
        // TODO(cmsweeney): get KS test working! Ensure sigmas are relatively
        // close to our expectations and the size of the overlap sets found so
        // far is relatively close.
        //        if (sigma_hat < sigma_max) {
        if (KolmogorovSmirnoffWrapper(current_model,
                                      valid_models[i],
                                      common_indices)) {
          overlap_count.push_back(common_indices.size());
          int min_overlap = *std::min_element(overlap_count.begin(),
                                              overlap_count.end());
          int max_overlap = *std::max_element(overlap_count.begin(),
                                              overlap_count.end());
          // If the sizes of the overlaps are not consistent then this is not a
          // good match!
          double size_difference =
              static_cast<double>((max_overlap - min_overlap))/min_overlap;
          if (size_difference > 0.05) {
            overlap_count.pop_back();
          } else {
            potential_inliers.push_back(common_indices);
            alpha_consistent_models->push_back(valid_models[i]);
            VLOG(0) << "\tpassed ks test!";
          }
        }
      }
    }
  }

  // Get intersection of inlier set from alpha consistent models.  Start with
  // all indices, and refine this set based on the inlier of the alpha
  // consistent models.
  inliers->resize(data.size());
  for (int i = 0; i < inliers->size(); i++)
    inliers->at(i) = i;

  for (int i = 0; i < potential_inliers.size(); i++) {
    std::sort(potential_inliers[i].begin(), potential_inliers[i].end());
    IntersectInPlace(inliers, potential_inliers[i]);
  }
}

template<class Datum, class Model>
void Recon<Datum, Model>::ComputeMeanInliers(
    const std::vector<Datum>& data,
    const std::vector<int>& inliers,
    std::vector<ModelResiduals>* models,
    std::vector<Datum>* mean_inliers) {
  // Create a temp holder for the mean of the data points.
  ModelResiduals mean;
  mean.reserve(inliers.size());
  for (int index : inliers)
    mean.push_back(ResidualDataPoint(index, 0.0));
  std::sort(mean.begin(), mean.end(), ResidualDataPoint::SortByIndex);

  // For each model, we add the residual of the inlier to our mean model.
  for (ModelResiduals& model : *models) {
    std::sort(model.begin(), model.end(), ResidualDataPoint::SortByIndex);
    for (int i = 0; i < inliers.size(); i++) {
      mean[i].SetResidual(mean[i].GetResidual() +
                          model[i].GetResidual());
    }
  }
  // Sort the mean residuals.
  std::sort(mean.begin(), mean.end(), ResidualDataPoint::SortByNormResidual);

  // Add (in sorted order) the data points to the mean inliers.
  mean_inliers->clear();
  mean_inliers->reserve(inliers.size());
  for (ResidualDataPoint rdp : mean) {
    mean_inliers->push_back(data[rdp.GetIndex()]);
  }
}

template<class Datum, class Model>
void Recon<Datum, Model>::GenerateProsacModels(
    const std::vector<Datum>& data,
    const Estimator<Datum, Model>& estimator,
    int num_models,
    std::vector<ModelResiduals>* models) {
  ProsacSampler<Datum> prosac_sampler(min_sample_size_);
  for (int i = 0; i < num_models; i++) {
    std::vector<Datum> prosac_subset;
    if (!prosac_sampler.Sample(data, &prosac_subset)) {
      prosac_sampler.SetSampleNumber(i + 1);
      continue;
    }

    Model prosac_model;
    // TODO(cmsweeney): Estimate models using non-minimal samples. Unclear what
    // the number of samples is for non-minimal model estimation.
    if (!estimator.EstimateModelNonminimal(prosac_subset, &prosac_model))
      continue;

    VLOG(0) << "proac model = " << prosac_model.m << ", " << prosac_model.b;
    // Get residuals
    std::vector<double> current_residuals = estimator.Residuals(data,
                                                                prosac_model);
    // Create current residual model.
    ModelResiduals current_model;
    current_model.reserve(current_residuals.size());
    for (int i = 0; i < current_residuals.size(); i++)
      current_model.push_back(ResidualDataPoint(i, current_residuals[i]));
    models->push_back(current_model);
  }
}

template<class Datum, class Model>
void Recon<Datum, Model>::FindBestInlierSet(
    const std::vector<Datum>& data,
    const std::vector<ModelResiduals>& models,
    std::vector<Datum>* best_data) {
  // Find the minimum n in which all models are alpha consistent.
  int n;
  std::vector<int> common(data.size(), 0);
  for (n = 0; n < data.size(); n++) {
    // Set each of the model points as visited.
    for (int i = 0; i < models.size(); i++) {
      common[models[i][n].GetIndex()]++;
    }
    // See how many points have been visited by all models.
    int common_elements = 0;
    for (int i = 0; i < common.size(); i++) {
      // VLOG(0) << "common_elements[" << i << "] = " << common[i];
      if (common[i] >= models.size())
        common_elements++;
    }
    // VLOG(0) << "num common elements = " << common_elements;
    if (static_cast<double>(common_elements)/(n + 1.0) >= alpha*alpha)
      break;
  }
  // n is currently set to the value which broke alpha consistency so we should
  // increase it by 1 to reflect the size of the overlap (as opposed to the
  // index).
  n++;
  VLOG(0) << "n = " << n;
  CHECK_GE(n, min_sample_size_) << "the overlap of best inliers was too small!";

  // Find the intersection of all models for the first n points. First, set the
  // indices to the first n points of the first model, then intersect this with
  // the rest of the models.
  std::vector<int> best_indices(n);
  for (int i = 0; i < n; i++) {
    best_indices[i] = models[0][i].GetIndex();
  }
  std::sort(best_indices.begin(), best_indices.end());

  // Intersect with the rest of the models.
  for (int i = 1; i < models.size(); i++) {
    // Fetch indices from model.
    std::vector<int> indices(n);
    for (int j = 0; j < n; j++)
      indices[j] = models[i][j].GetIndex();
    std::sort(indices.begin(), indices.end());
    IntersectInPlace(&best_indices, indices);
  }
  CHECK_GE(best_indices.size(), min_sample_size_)
      << "intersection of best inliers resulted in too small of a sample!";

  // Put the intersection of inliers into a vector of Datum.
  for (int i = 0; i < n; i++) {
    best_data->push_back(data[best_indices[i]]);
  }
}

template<class Datum, class Model>
bool Recon<Datum, Model>::Estimate(const std::vector<Datum>& data,
                                   const Estimator<Datum, Model>& estimator,
                                   Model* best_model) {
  std::vector<ModelResiduals> alpha_consistent_models;
  std::vector<int> inliers;
  FindAlphaConsistentModels(data,
                            estimator,
                            &alpha_consistent_models,
                            &inliers);
  VLOG(0) << "found alpha consistent models";
  std::vector<Datum> mean_inliers;
  ComputeMeanInliers(data,
                     inliers,
                     &alpha_consistent_models,
                     &mean_inliers);

  VLOG(0) << "computed mean of models";
  VLOG(0) << "inliers of size " << inliers.size();
  std::vector<ModelResiduals> prosac_models;
  GenerateProsacModels(mean_inliers,
                       estimator,
                       20,
                       &prosac_models);
  VLOG(0) << "generated prosac models";

  // Sort all models by their residuals.
  for (int i = 0; i < prosac_models.size(); i++)
    std::sort(prosac_models[i].begin(),
              prosac_models[i].end(),
              ResidualDataPoint::SortByNormResidual);

  std::vector<Datum> best_inlier_set;
  FindBestInlierSet(data,
                    prosac_models,
                    &best_inlier_set);

  // Effectively establish a random sampling of the inliers since they are
  // returned in index-sorted order.
  std::random_shuffle(best_inlier_set.begin(), best_inlier_set.end());
  estimator.EstimateModelNonminimal(best_inlier_set, best_model);
  VLOG(0) << "best model = " << best_model->m << ", " << best_model->b;
  return true;
}
}  // namespace theia
#endif  // SOLVERS_RECON_H_
