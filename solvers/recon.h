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

#ifndef SOLVERS_RECON_H_
#define SOLVERS_RECON_H_

#include <algorithm>
#include <math.h>
#include <cstdlib>
#include <math.h>
#include <memory>
#include <unordered_map>
#include <vector>
#include <iostream>

#include "math/probability/kolmogorov_smirnoff.h"
#include "solvers/estimator.h"
#include "solvers/inlier_support.h"
#include "solvers/prosac_sampler.h"
#include "solvers/random_sampler.h"
#include "solvers/sample_consensus_estimator.h"

namespace solvers {
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
  //   is determined.
  Recon(int min_sample_size,
        int min_consistent_models)
      : min_sample_size_(min_sample_size),
        min_consistent_models_(min_consistent_models),
        alpha(0.95),
        num_nonminimal_models(20) {}

  Recon(int min_sample_size)
      : Recon<Datum, Model>(min_sample_size, 5) {}

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


  // Holding struct for residuals and the indices of the data points.
  struct ResidualDataPoint {
    double residual;
    int index;
    int sorted_index;
    ResidualDataPoint(const double r, const int i, const int si)
        : residual(r), index(i), sorted_index(si) {}
    ResidualDataPoint() {}
    // beginning of the vector.
  };

  // Performs the Kolmogorov-Smirnov test to ensure that the two residual sets
  // are similar for the first n data points sampled.
  bool KSTest(const std::vector<ResidualDataPoint>& residual1,
              const std::vector<ResidualDataPoint>& residual2,
              int n);

  // Private Class for holding the model residuals in an intelligent way. This
  // allows for us to keep track of residuals, sorted order, and check the alpha
  // consistency easily.
  class ModelResiduals {
   public:

    // Used to sort model residuals such that the lowest residual is at the
    // beginning.
    struct SortResiduals {
      bool operator()(const ResidualDataPoint& a,
                      const ResidualDataPoint& b) {
        return a.residual < b.residual;
      }
    } residual_sorter;

    // Constructor. Stores the residuals and sorts the data. It also creates an
    // efficient lookup table where the original index is the key and a pointer
    // to the residual data point is the value.
    ModelResiduals(const std::vector<double>& residuals) {
      residual_data_points.reserve(residuals.size());
      for (int i = 0; i < residuals.size(); i++) {
        // Create the residual data points. Note the sorted index is -1 until
        // the list is actually sorted.
        residual_data_points.push_back(ResidualDataPoint(residuals[i], i, -1));
      }
      // Sort data.
      std::sort(residual_data_points.begin(),
                residual_data_points.end(),
                residual_sorter);
      
      // Update sorted index.
      sorted_residuals.reserve(residuals.size());
      for (int i = 0; i < residuals.size(); i++) {
        residual_data_points[i].sorted_index = i;
        // Add index to lookup.
        index_lookup[residual_data_points[i].index] = &residual_data_points[i];
        sorted_residuals.push_back(residual_data_points[i].residual);
      }
    }

    // Test alpha consistency. If some n exists such that the common
    // intersection is greater than alpha squared, then they are alpha
    // consistent. The alpha test also includes the sigma test of Eq 9.
    bool AlphaConsistentWith(const ModelResiduals& other_model,
                             double alpha,
                             int* n) const {
      double alpha_sq = alpha*alpha;
      const int max_iterations =
          std::min(this->residual_data_points.size(),
                   other_model.residual_data_points.size());
      int num_common_members = 0;
      // See how many members in the first n are in common.
      // TODO(cmsweeney): Perhaps start this at a higher number?
      bool alpha_test = false;
      for (*n = 0; *n < alpha_sq*max_iterations - 1; (*n)++) {
        // Fetch the common data point in the other model's residuals. Residuals
        // are stored by original index in the lookup.
        const ResidualDataPoint* other_residual =
            other_model.index_lookup.find(
                residual_data_points[*n].index)->second;

        CHECK_EQ(other_residual->index, residual_data_points[*n].index)
            << "Found bad data!";

        // If the sorted index of the other data point is < n, then we know it
        // is within the first n data points, so it is a common inlier.
        if (other_residual->sorted_index <= *n)
          num_common_members++;

        // Do the same test, but the other way around. Only do this test if the
        // two values are not the same (because that would add to common members
        // twice)!
        const ResidualDataPoint* this_residual = index_lookup.find(
            other_model.residual_data_points[*n].index)->second;

        CHECK_EQ(this_residual->index,
                 other_model.residual_data_points[*n].index) << "Found bad data!";

        if (this_residual->index != other_residual->index &&
            this_residual->sorted_index <= *n)
          num_common_members++;

        const double common_inlier_ratio =
            static_cast<double>(num_common_members)/(*n + 1.0);

        // If the two models are alpha consistent, break and proceed to the
        // sigma test.
        if (common_inlier_ratio >= alpha_sq && *n > 3) {
          (*n)++;
          VLOG(1) << "num common members = " << num_common_members
                  << " of " << *n;
          VLOG(1) << "common inlier ratio = " << common_inlier_ratio;
          return true;
        }
      }
      return false;
    }

    // Holds all of the residuals in sorted order.
    std::vector<ResidualDataPoint> residual_data_points;

    // Sorted residuals stored by themselves for quick access!
    std::vector<double> sorted_residuals;
    
    // A lookup table so that you can find the residual data point from the
    // original index.
    std::unordered_map<int, ResidualDataPoint*> index_lookup;

    Model model;
  };

};

template<class Datum, class Model>
bool Recon<Datum, Model>::Estimate(const std::vector<Datum>& data,
                                   const Estimator<Datum, Model>& estimator,
                                   Model* best_model) {
  int num_consistent_models = 0;
  RandomSampler<Datum> random_sampler(min_sample_size_);
  // Repeat until we have the minimum number of consistent models needed to
  // determine a convergence.
  std::vector<std::unique_ptr<ModelResiduals> > valid_models;
  std::vector<ModelResiduals*> alpha_consistent_models;
  while (num_consistent_models < min_consistent_models_) {
    // Estimate a model from a random sample.
    std::vector<Datum> data_subset;
    if(!random_sampler.Sample(data, &data_subset))
      continue;
    Model temp_model;
    if (!estimator.EstimateModel(data_subset, &temp_model))
      continue;

    // Early exit if the model is not valid.
    if (!estimator.ValidModel(temp_model))
      continue;

    VLOG(0) << "temp_model = " << temp_model.m  << ", "
            << temp_model.b
            << "\nfrom (" << data_subset[0].x << ", " << data_subset[0].y
            << ") and (" << data_subset[1].x << ", " << data_subset[1].y << ")";

    // Determine residuals.
    std::vector<double> temp_residuals = estimator.Residuals(data, temp_model);

    // Add residuals to the model residual holder. This will sort the data by
    // the residuals, which will be used for alpha testing.
    valid_models.push_back(
        std::unique_ptr<ModelResiduals>(new ModelResiduals(temp_residuals)));
    ModelResiduals* current_model = valid_models.back().get();
    current_model->model = temp_model;
    alpha_consistent_models.clear();
    alpha_consistent_models.push_back(current_model);
    for (int i = 0; i < valid_models.size() - 1; i++) {
      int n;
      VLOG(0) << "iteration " << i << " of alpha test";
      if (current_model->AlphaConsistentWith(*valid_models[i],
                                             alpha,
                                             &n)) {
        VLOG(0) << "model " << valid_models[i]->model.m
                << ", " << valid_models[i]->model.b
                << " is alpha consistent at " << n;
        double sigma_hat = 1.4286*(1.0 + 5.0/(n - min_sample_size_))*
            sqrt(valid_models[i]->sorted_residuals[n/2]);
        
        if (sigma_hat < sigma_max ||
            math::probability::KolmogorovSmirnoffTest(
                current_model->sorted_residuals,
                valid_models[i]->sorted_residuals,
                n)) {
          VLOG(0) << "passed KS test!";
          VLOG(0) << "alpha models size = " << alpha_consistent_models.size();
          // Add to set of alpha consistent models.
          alpha_consistent_models.push_back(valid_models[i].get());
          // If we have enough alpha consistent models, quit!
          if (alpha_consistent_models.size() >= min_consistent_models_)
            break;
        }
      }
    }
    // If we exited the previous loop with enough models, break while loop
    if (alpha_consistent_models.size() >= min_consistent_models_)
      break;
  }

  for (ModelResiduals* acm : alpha_consistent_models)
    VLOG(0) << "alpha consistent model = " << acm->model.m << ", " << acm->model.b;
  
  // Collect the means of residuals from the alpha consistent models.
  std::vector<double> mean_residuals(data.size());
  for (int i = 0; i < data.size(); i++) {
    for (ModelResiduals* current_model : alpha_consistent_models) {
      mean_residuals[i] += current_model->index_lookup[i]->residual;
      CHECK_EQ(i, current_model->index_lookup[i]->index);
    }
    mean_residuals[i] /= alpha_consistent_models.size();
  }

  // Sort inliers by the mean residuals.
  ModelResiduals mean_residual_model(mean_residuals);
  VLOG(0) << "mean residuals created";
  // Generate M (= 20) non-minimal Models using PROSAC sampling over sorted
  // inliers.
  std::vector<ModelResiduals> prosac_models;
  prosac_models.reserve(num_nonminimal_models*num_nonminimal_models);
  ProsacSampler<ResidualDataPoint> prosac_sampler(min_sample_size_);
  for (int i = 0; i < num_nonminimal_models; i++) {
    std::vector<ResidualDataPoint> prosac_subsets;
    // Sample the sorted data using PROSAC sampling.
    prosac_sampler.Sample(mean_residual_model.residual_data_points,
                          &prosac_subsets);
    // Load data from the sample.
    std::vector<Datum> prosac_data(min_sample_size_);
    for (int j = 0; j < min_sample_size_; j++) {
      prosac_data[j] = data[prosac_subsets[j].index];
    }

    // Generate model from PROSAC sampling and save.
    Model prosac_model;
    estimator.EstimateModelNonminimal(prosac_data, &prosac_model);
    std::vector<double> temp_residuals = estimator.Residuals(data, prosac_model);
    prosac_models.push_back(ModelResiduals(temp_residuals));
    VLOG(0) << "prosac_model = " << prosac_model.m  << ", "
            << prosac_model.b;
  }

  // Find the minimum n such that all pairs are alpha consistent.
  std::vector<int> pairwise_n;
  for (int i = 0; i < num_nonminimal_models; i++) {
    for (int j = i + 1; j < num_nonminimal_models; j++){
      int n;
      prosac_models[i].AlphaConsistentWith(prosac_models[j],
                                           alpha,
                                           &n);
      pairwise_n.push_back(n);
    }
  }

  // Re-estimate final model using best inlier set.
  // TODO(cmsweeney): It is unclear what the "best" set is at this point. All
  // the sets should have very high quality sets for data points < n.
  std::vector<double> best_residuals(data.size());
  for (int i = 0; i < data.size(); i++) {
    for (ModelResiduals& current_model : prosac_models) {
      best_residuals[i] += current_model.index_lookup[i]->residual;
    }
    best_residuals[i] /= prosac_models.size();
  }
  ModelResiduals best_residuals_model(best_residuals);

  std::vector<Datum> final_inliers(min_sample_size_);
  for (int i = 0; i < min_sample_size_; i++)
    final_inliers[i] = data[best_residuals_model.residual_data_points[i].index];
  estimator.EstimateModel(final_inliers, best_model);
  VLOG(0) << "best model = " << best_model->m << ", " << best_model->b;
  // estimator.RefineModel(consensus_set, best_model);

  //inliers_.resize(data.size());

  return true;
}

}  // namespace solvers

#endif  // SOLVERS_RECON_H_
