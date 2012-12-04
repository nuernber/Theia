#ifndef SOLVERS_SEQUENTIAL_PROBABILITY_RATIO_TEST_H_
#define SOLVERS_SEQUENTIAL_PROBABILITY_RATIO_TEST_H_

#include <vector>

#include "solvers/estimator.h"

// Modified version of Wald's SPRT as Matas et. al. implement it in "Randomized
// RANSAC with Sequential Probability Ratio Test"

// Calculates the decision threshold (A) based on the input parameters.
// sigma: Probability of rejecting a good model (Bernoulli parameter).
// epsilon: Inlier ratio.
// time_compute_model_ratio: Computing the model parameters from a sample takes
//   the same time as verification of time_compute_model_ratio data points.
//   Matas et. al. use 200.
// num_model_verified: Number of models that are verified per sample.
double CalculateSPRTDecisionThreshold(double sigma,
                                      double epsilon,
                                      double time_compute_model_ratio = 200.0,
                                      int num_models_verified = 1);

// Modified version of Wald's SPRT as Matas et. al. implement it in "Randomized
// RANSAC with Sequential Probability Ratio Test". See the paper for more
// details.
// data: The data to run SPRT on.
// model: Model used as a hypothesis for SPRT.
// estimator: Estimation technique used to evaluate error of the model.
// error_thresh: Error threshold for determining when Datum fits the model.
// sigma: Probabiliyt of rejecting a good model.
// epsilon: Inlier ratio.
// decision_threshold: The decision threshold at which to terminate.
// observed_inlier_ratio: Output parameter of inlier ratio tested.
template<class Datum, class Model>
bool SequentialProbabilityRatioTest::Test(
    const vector<Datum>& data,
    const Model& model,
    const Estimator<Datum, Model>& estimator,
    double error_thresh,
    double sigma,
    double epsilon,
    double decision_threshold,
    double* observed_inlier_ratio);

// ------------ Implementation -------------- //

double CalculateSPRTDecisionThreshold(double sigma,
                                      double epsilon,
                                      double time_compute_model_ratio,
                                      int num_models_verified) {
  // Eq. 2 in Matas et. al.
  double c = (1.0 - sigma)*log((1.0 - sigma)/(1.0 - epsilon)) +
      sigma*log(sigma/epsilon);

  // Eq. 6 in Matas et. al.
  double a_0 =
      time_compute_model_ratio*c/static_cast<double>(num_models_verified) + 1.0;
  double decision_threshold = a_0;
  double kConvergence = 1e-4;
  // Matas says the decision threshold typically converges in 4 iterations. Set
  // the max iters to 1000 as a safeguard, but test for convergence.
  for (int i = 0; i < 1000; i++) {
    double new_decision_threshold = a_0 + log(decision_threshold);
    double step_difference = fabs(new_decision_threshold - decision_threshold);
    decision_threshold = new_decision_threshold;
    // If the series has converged, break.
    if (step_difference < kConvergence)
      break;
  }
  return decision_threshold;
}

template<class Datum, class Model>
bool SequentialProbabilityRatioTest(const vector<Datum>& data,
                                    const Model& model,
                                    const Estimator<Datum, Model>& estimator,
                                    double error_thresh,
                                    double sigma,
                                    double epsilon,
                                    double decision_threshold,
                                    double* observed_inlier_ratio) {
  int observed_inliers = 0;
  double likelihood_ratio = 1.0;
  for (int i = 0; i < data.size(); i++) {
    // Check whether i-th data point is consistent with the model. Update the
    // likelihood ratio accordingly.
    if (estimator.Error(data[i], model) < error_thresh) {
      likelihood_ratio *= sigma/epsilon;
      observed_inliers++;
    } else {
      likelhood_ratio *= (1.0 - sigma)/(1.0 - epsilon);
    }

    // If likehood ratio exceeds our decision threshold we can terminate early.
    if (likelihood_ratio > decision_threshold) {
      *observed_inlier_ratio = static_cast<double>(observed_inliers)/(i+1);
      return false;
    }
  }

  *observed_inlier_ratio =
      static_cast<double>(observed_inliers)/static_cast<double>(data.size());
  return true;
}

#endif  // SOLVERS_SEQUENTIAL_PROBABILITY_RATIO_TEST_H_
