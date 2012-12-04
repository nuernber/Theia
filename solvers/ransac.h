#ifndef SOLVERS_RANSAC_H_
#define SOLVERS_RANSAC_H_

#include <math.h>
#include <cstdlib>
#include <vector>

#include "solvers/estimator.h"

using std::vector;

#define RANSAC_PARAM_INVALID -1

namespace solvers {

template<class Datum, class Model>
class Ransac {
 public:
  // min_sample_size: the minimum number of samples needed to estimate a model
  // termination_inlier_ratio: The percentage of inliers at which the estimated
  //   model is deemed successful for early termination
  // max_iters: Maximum number of iterations to run RANSAC. To set the number
  //   of iterations based on the outlier probability, use SetMaxIters.
  explicit Ransac(int min_sample_size,
                  double termination_inlier_ratio,
                  int max_iters)
      : min_sample_size_(min_sample_size),
        termination_inlier_ratio_(termination_inlier_ratio),
        max_iters_(max_iters),
        termination_num_inliers(RANSAC_PARAM_INVALID) {}

  // See MaxItersFromOutlierProb for parameters.
  Ransac(int min_sample_size,
         double termination_inlier_ratio,
         double outlier_probability,
         double fail_probability = 0.99)
      : min_sample_size_(min_sample_size),
        termination_inlier_ratio_(termination_inlier_ratio),
        max_iters_(MaxItersFromOutlierProb(min_sample_size,
                                           outlier_probability,
                                           fail_probability)),
        termination_num_inliers(RANSAC_PARAM_INVALID) {}

  // Computes the best-fitting model using RANSAC. Returns false if RANSAC
  // calculation fails and true (with the best_model output) if successful.
  // Params:
  //   data: the set from which to sample
  //   estimator: The estimator used to estimate the model based on the Datum
  //     and Model type
  //   error_thresh: The error threshold for determining when datum fits a model
  //   best_model: The output parameter that will be filled with the best model
  //     estimated from RANSAC
  bool Compute(const vector<Datum>& data,
               const Estimator<Datum, Model>& estimator,
               double error_thresh,
               Model* best_model);

  // Returns a bool vector with true for inliers, false for outliers.
  vector<bool> GetInliers() { return inliers_; }

  // Count the number of inliers.
  int GetNumInliers() {
    int num_inliers = 0;
    for (bool inlier : inliers_) {
      if (inlier)
        num_inliers++;
    }
    return num_inliers;
  }

  // The following variable(s) can be set explicitly. E.g. pose estimation often
  // requires 12 inliers rather than a specific ratio. If set, these variables
  // will be used as opposed to the corresponding variables set in the
  // constructor.
  //
  // Number of inliers needed to terminate RANSAC (i.e. not a ratio).
  int termination_num_inliers;

 private:
  // Set the max iterations based on Eq. 4.18 in Hartley & Zisserman.
  //   min_sample_size: the min number of samples required to estimate a model.
  //   outlier_probability: prob that a given data point is an outlier
  //   fail_probability: prob that at least one sample has no outliers
  //     (typically set to .99)
  int MaxItersFromOutlierProb(int min_sample_size,
                              double outlier_probability,
                              double fail_probability = 0.99) {
    return ceil(log(1 - fail_probability) /
                log(1.0 - pow(1.0 - outlier_probability, min_sample_size)));
  }

  vector<bool> inliers_;

  // Max number of iterations to perform before terminating RANSAC.
  int max_iters_;

  // Minimum samples size for an estimated model.
  int min_sample_size_;

  // The percentage of inliers in which RANSAC terminates.
  double termination_inlier_ratio_;
};

template<class Datum, class Model>
bool Ransac<Datum, Model>::Compute(const vector<Datum>& data,
                                   const Estimator<Datum, Model>& estimator,
                                   double error_thresh,
                                   Model* best_model) {
  if (min_sample_size_ > data.size())
    return false;

  // Set termination_num_inliers based on the inlier ratio if not manually set.
  termination_num_inliers = (termination_num_inliers == RANSAC_PARAM_INVALID) ?
      ceil(termination_inlier_ratio_ * data.size()) : termination_num_inliers;

  int best_consensus_size = 0;
  for (int iters = 0; iters < max_iters_; iters++) {
    // Randomly select subset.
    vector<Datum> data_random_subset;
    for (int i = 0; i < min_sample_size_; i++) {
      // Generate a random index to sample. There is a small chance that this
      // will result in the same index multiple times in the subset. This is OK
      // because it will result in a degenerate model that will be thrown out.
      int random_index = rand() % data.size();

      // Add random point to the sample subset.
      data_random_subset.push_back(data[random_index]);
    }

    // Estimate model from random subset. Skip to next iteration if the model
    // fails to estimate.
    Model temp_model;
    if (!estimator.EstimateModel(data_random_subset, &temp_model))
      continue;

    // Count all inliers for the estimated model.
    vector<Datum> temp_consensus_set;
    for (int i = 0; i < data.size(); i++) {
      if (estimator.Error(data[i], temp_model) < error_thresh) {
        temp_consensus_set.push_back(data[i]);
      }
    }

    // Update best model if error is the best we have seen.
    if (temp_consensus_set.size() > best_consensus_size) {
      *best_model = temp_model;
      best_consensus_size = temp_consensus_set.size();

      // If the inlier termination criterion is met, re-estimate the model
      // based on all inliers and return;
      if (best_consensus_size >= termination_num_inliers) {
        estimator.RefineModel(temp_consensus_set, best_model);
        break;
      }
    }
  }

  inliers_.resize(data.size());
  for (int i = 0; i < data.size(); ++i) {
    inliers_[i] = estimator.Error(data[i], *best_model) < error_thresh;
  }
  return true;
}
}  // namespace solvers

#endif  // SOLVERS_RANSAC_H_
