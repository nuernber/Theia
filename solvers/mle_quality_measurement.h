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

#ifndef SOLVERS_MLE_QUALITY_MEASUREMENT_H_
#define SOLVERS_MLE_QUALITY_MEASUREMENT_H_

#include <glog/logging.h>
#include <cmath>
#include <limits>
#include <vector>

#include "math/distribution.h"
#include "solvers/quality_measurement.h"

namespace theia {
// Define the quality metric according to Guided MLE from "Guided sampling and
// consensus for motion estimation" by Tordoff and Murray
class MLEQualityMeasurement : public QualityMeasurement {
 public:
  // Params:
  //  inlier_dist: Distribution of inlier noise. Typically a gaussian mixture.
  //  outlier_dist: Distribution of outlier noise. Typically a uniform
  //    distribution over the search radius (in pixels).
  //  confidence_threshold: if the confidence of the data is greater then this
  //    then MLESAC will terminate.
  //  inlier_prob: An initial guess for the inlier probability (range 0 to 1).
  MLEQualityMeasurement(const Distribution& inlier_dist,
                        double uniform_prob,
                        double confidence_threshold,
                        double inlier_prob)
      : inlier_dist_(inlier_dist),
        outlier_prob_(uniform_prob),
        confidence_threshold_(confidence_threshold),
        inlier_probability_(inlier_prob) {}

  ~MLEQualityMeasurement() {}

  // Given the residuals, assess a quality metric for the data. Returns the
  // quality assessment and outputs a vector of bools indicating the inliers.
  double Calculate(const std::vector<double>& residuals,
                   std::vector<bool>* inliers) {
    inliers->resize(residuals.size(), false);

    double estimated_inlier_prob = inlier_probability_;
    const int kNumEMIterations = 5;
    double arglog;
    double gamma_sum;
      for (int i = 0; i < kNumEMIterations; i++) {
      arglog = 0.0;
      gamma_sum = 0.0;
      for (int i = 0; i < residuals.size(); i++) {
        const double& r = residuals[i];
        double pr_inlier = inlier_dist_.eval(r) * estimated_inlier_prob;
        double pr_outlier = outlier_prob_ * (1.0 - estimated_inlier_prob);
        arglog += log(pr_inlier + pr_outlier);
        gamma_sum += pr_inlier / (pr_inlier + pr_outlier);
      }
      estimated_inlier_prob = gamma_sum / residuals.size();
    }
    const double kSmallNumber = 1e-6;
    if (arglog == 0.0)
      arglog = kSmallNumber;
    return -arglog;
  }

  // Given two quality measurements, determine which is betters. Note that
  // larger is not always better! Returns true if quality1 is of higher
  // quality than quality2.
  bool Compare(const double quality1, const double quality2) {
    return quality1 < quality2;
  }

  // Return true if the quality passed in is high enough to terminate the
  // sampling consensus.
  bool SufficientlyHighQuality(const double quality) {
    return quality <= confidence_threshold_;
  }

 private:
  // Threshold for determining whether MLE estimate is good enough.
  double confidence_threshold_;

  // Distribution of inlier data.
  const Distribution& inlier_dist_;

  // Distribution of outlier data.
  const double outlier_prob_;

  // Estimated inlier probability of the data.
  const double inlier_probability_;
};
}       // namespace theia
#endif  // SOLVERS_MLE_QUALITY_MEASUREMENT_H_
