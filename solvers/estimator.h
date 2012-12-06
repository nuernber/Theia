#ifndef SOLVERS_ESTIMATOR_H_
#define SOLVERS_ESTIMATOR_H_

#include <vector>

namespace solvers {
// Templated class for estimating a model for RANSAC. This class is purely a
// virtual class and should be implemented for the specific task that RANSAC is
// being used for. Two methods must be implemented: EstimateModel and Error. All
// other methods are optional, but will likely enhance the quality of the RANSAC
// output.
//
// NOTE: RANSAC, ARRSAC, and other solvers work best if Datum and Model are
// lightweight classes or structs.

template <class Datum, class Model>
class Estimator {
 public:
  Estimator() {}
  virtual ~Estimator() {}

  // Given a set of data points, estimate the model. Users should implement this
  // function appropriately for the task being solved. Returns true for
  // successful model estimation (and outputs model), false for failed
  // estimation.
  virtual bool EstimateModel(const std::vector<Datum>& data,
                             Model* model) const = 0;

  // Refine the model based on an updated subset of data, and a pre-computed
  // model. Can be optionally implemented.
  virtual bool RefineModel(const std::vector<Datum>& data, Model* model) const {
    return true;
  };

  // Given a model and a data point, calculate the error. Users should implement
  // this function appropriately for the task being solved.
  virtual double Error(const Datum& data, const Model& model) const = 0;

  // Returns the set inliers of the data set based on the error threshold
  // provided.
  std::vector<bool> GetInliers(const std::vector<Datum>& data,
                               const Model& model,
                               double error_threshold) const {
    std::vector<bool> inliers;
    for (const Datum& data_point : data)
      inliers.push_back(Error(data, model) < error_threshold);
    return inliers;
  }

  // Returns the number inliers of the data set based on the error threshold
  // provided.
  int GetNumInliers(const std::vector<Datum>& data,
                    const Model& model,
                    double error_threshold) const {
    int num_inliers = 0;
    for (const Datum& data_point : data)
      if (Error(data, model) < error_threshold)
        num_inliers++;
    return num_inliers;
  }
};

}  // namespace solvers

#endif  // SOLVERS_ESTIMATOR_H_
