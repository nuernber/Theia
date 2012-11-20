#ifndef SOLVERS_RANSAC_H
#define SOLVERS_RANSAC_H

#include <cstdlib>
#include <math.h>
#include <vector>

#include <iostream>

using namespace std;

#define RANSAC_PARAM_INVALID -1

namespace solvers {
// Templated class for estimating a model for RANSAC. This class is purely a
// virtual class and should be implemented for the specific task that RANSAC is
// being used for. Two methods must be implemented: EstimateModel and Error. All
// other methods are optional, but will likely enhance the quality of the
// RANSAC output.
template <class Datum, class Model>
class Estimator {
 public:
  Estimator() {}
  virtual ~Estimator() {}

  // Given a set of data points, estimate the model. Users should implement this
  // function appropriately for the task being solved. Returns true for
  // successful model estimation (and outputs model), false for failed
  // estimation.
  virtual bool EstimateModel(const vector<Datum>& data, Model* model) const = 0;

  // Refine the model based on an updated subset of data, and a pre-computed
  // model. Can be optionally implemented.
  virtual bool RefineModel(const vector<Datum>& data, Model* model) const {
    return true;
  };

  // Given a model and a data point, calculate the error. Users should implement
  // this function appropriately for the task being solved.
  virtual double Error(const Datum& data, const Model& model) const = 0;
};

template<class Datum, class Model>
class Ransac {
 public:
  // Params:
  //   estimator: The estimator (derived from the above Estimator class) used
  //     to estimate the model based on the Datum and Model type.
  //   min_sample_size: The minimum number of samples used to fit the model.
  //   error_thresh: The error threshold for determining when datum fits a model
  //   termination_inlier_ratio: Terminate RANSAC if we achieve a model with a
  //     percentage of inliers higher than this.
  //   max_iters: Maximum number of iterations to run RANSAC. To set the number
  //     of iterations based on the outlier probability, use SetMaxIters.
  explicit Ransac(Estimator<Datum, Model>* estimator,
                  int min_sample_size,
                  double error_thresh,
                  double termination_inlier_ratio,
                  int max_iters)
      : estimator_(estimator),
        min_sample_size_(min_sample_size),
        error_thresh_(error_thresh),
        termination_inlier_ratio_(termination_inlier_ratio),
        max_iters_(max_iters),
        termination_num_inliers(RANSAC_PARAM_INVALID) {}

  // Add point to be fitted to the model. Random samples come solely from data,
  // points.
  void AddDataPoint(const Datum& data) { data_.push_back(data); }

  // Computes the best-fitting model using RANSAC. Returns false if RANSAC
  // calculation fails and true (with the best_model output) if successful.
  bool Compute(Model* best_model);

  // Returns a bool vector with true for inliers, false for outliers.
  vector<bool> GetInliers() const;

  // Set the max iterations based on Eq. 4.18 in Hartley & Zisserman.
  // - fail_probability = prob that at least one sample has no outliers
  //     (typically set to .99)
  // - outlier_probability = prob that a given data point is an outlier
  void SetMaxIters(double outlier_probability,
                   double fail_probability = 0.99) {
    max_iters_ =
        ceil(log(1 - fail_probability) /
             log(1.0 - pow(1.0 - outlier_probability, min_sample_size_)));
  }

  // The following variables can be set explicitly. E.g. pose estimation often
  // requires 12 inliers rather than a specific ratio. If set, these variables
  // will be used as opposed to the corresponding variables set in the
  // constructor.
  //
  // Number of inliers needed to terminate RANSAC (i.e. not a ratio).
  int termination_num_inliers;

 private:
  // Container for all data points.
  vector<Datum> data_;

  // Estimator for estimating a model from a (random) subset of the data.
  Estimator<Datum, Model>* estimator_;

  Model best_model_;
  
  // Error tolerance for a single measurement to be considered an inlier.
  double error_thresh_;

  // Max number of iterations to perform before terminating RANSAC.
  int max_iters_;

  // Minimum samples size for an estimated model.
  int min_sample_size_;

  // The percentage of inliers in which RANSAC terminates.
  double termination_inlier_ratio_;
};

template<class Datum, class Model>
bool Ransac<Datum, Model>::Compute(Model* best_model) {

  if (min_sample_size_ > data_.size())
    return false;

  // Set termination_num_inliers based on the inlier ratio if not manually set.
  termination_num_inliers = (termination_num_inliers == RANSAC_PARAM_INVALID) ?
      ceil(termination_inlier_ratio_ * data_.size()) : termination_num_inliers;

  int best_consensus_size = 0;
  for (int iters = 0; iters < max_iters_; ++iters) {
    // Randomly select subset.
    vector<Datum> data_random_subset;
    for (int i = 0; i < min_sample_size_; ++i) {
      // Generate a random index to sample. There is a small chance that this
      // will result in the same index multiple times in the subset. This is OK
      // because it will result in a degenerate model that will be thrown out.
      int random_index = rand() % data_.size();

      // Add random point to the sample subset.
      data_random_subset.push_back(data_[random_index]);
    }

    // Estimate model from random subset. Skip to next iteration if the model
    // fails to estimate.
    Model temp_model;
    if (!estimator_->EstimateModel(data_random_subset, &temp_model))
      continue;

    // Find all inliers for the estimated model.
    vector<Datum> temp_consensus_set;
    for (const Datum& data_point : data_) {
      if (estimator_->Error(data_point, temp_model) < error_thresh_) {
        temp_consensus_set.push_back(data_point);
      }
    }

    // Update best model if error is the best we have seen.
    if (temp_consensus_set.size() > best_consensus_size) {
      *best_model = temp_model;
      best_consensus_size = temp_consensus_set.size();

      // If the inlier termination criterion is met, re-estimate the model based
      // on all inliers and return;
      if (best_consensus_size >= termination_num_inliers) {
        estimator_->RefineModel(temp_consensus_set, best_model);
        // Copy model locally (for use in other methods).
        best_model_ = *best_model;
        return true;
      }
    }
  }
  // Copy model locally (for use in other methods).
  best_model_ = *best_model;
  return true;
}

template<class Datum, class Model>
vector<bool> Ransac<Datum, Model>::GetInliers() const {
  vector<bool> inliers(data_.size(), false);
  for (int i = 0; i < data_.size(); ++i) {
    if (estimator_->Error(data_[i], best_model_) < error_thresh_) {
      inliers[i] = true;
    }
  }
}

}  // namespace solvers

#endif  // SOLVERS_RANSAC_H
