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

#ifndef SOLVERS_MLE_QUALITY_MEASUREMENT_H_
#define SOLVERS_MLE_QUALITY_MEASUREMENT_H_

#include <vector>
#include "solvers/quality_measurement.h"

namespace solvers {

// Define the quality metric according to MLE
class MLEQualityMeasurement : public QualityMeasurement {
 public:
  MLEQualityMeasurement(double terminating_thresh,
                        double inlier_ratio = 0.5)
      : terminating_threshold_(terminating_thresh),
        inlier_ratio_(inlier_ratio) {}
  ~MLEQualityMeasurement() {}

  // Given the residuals, assess a quality metric for the data. Returns the
  // quality assessment and outputs a vector of bools indicating the inliers.
  double Calculate(const std::vector<double>& residuals,
                   std::vector<bool>* inliers) {
    inliers->resize(residuals.size(), false);
    double mle = 0.0;
    double inlier_ratio_estimate = inlier_ratio_;
    for (int i = 0; i < residuals.size(); i++) {
      double pr_inlier;
      double pr_outlier;
      double confidence = pr_inlier/(pr_inlier + pr_outlier);

      // Count as an inlier if our confidence to this point is above the
      // terminating confidence.
      if (confidence > terminating_threshold) {
        inliers->at(i) = true;
      }

      mle += log(confidence);
    }
  }

  // Given two quality measurements, determine which is betters. Note that
  // larger is not always better! Returns true if quality1 is of higher quality
  // than quality2.
  bool Compare(const double quality1, const double quality2) {
    return quality1 < quality2;
  }

  // Return true if the quality passed in is high enough to terminate the
  // sampling consensus.
  bool SufficientlyHighQuality(const double quality) {
    return quality <= terminating_threshold_;
  }
 private:
  // Threshold for determining whether MLE estimate is good enough.
  double terminating_threshold_;
  // Initial estimate for the inlier ratio of the data.
  double inlier_ratio_;
};
}  // namespace solvers
#endif  // SOLVERS_MLE_QUALITY_MEASUREMENT_H_
