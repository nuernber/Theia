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
// ARE DISCLAIMED. IN NO EVENT SHALL CHRIS SWEENEY BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

namespace solvers {

// Define the quality metric according to Guided MLE from "Guided sampling and
// consensus for motion estimation" by Tordoff and Murray
class MLEQualityMeasurement : public QualityMeasurement {
 public:
  // Params:
  //  inlier_dist: Distribution of inlier noise. Typically a gaussian mixture.
  //  outlier_dist: Distribution of outlier noise. Typically a uniform
  //    distribution over the search radius (in pixels).
  //  confidence: The confidence of each measurement.
  //  confidence_thesh: All measurements with a confidence higher than this will
  //    be considered an inlier.
  MLEQualityMeasurement(const math::Distribution& inlier_dist,
                        const math::Distribution& outlier_dist,
                        const std::vector<double>& confidence,
                        double confidence_thresh)
      : inlier_dist_(inlier_dist),
        outlier_dist_(outlier_dist),
        confidence_(confidence),
        confidence_threshold_(confidence_thresh) {}

  ~MLEQualityMeasurement() {}

  // Given the residuals, assess a quality metric for the data. Returns the
  // quality assessment and outputs a vector of bools indicating the inliers.
  double Calculate(const std::vector<double>& residuals,
                   std::vector<bool>* inliers) {
    const double kInfinity = 1e24;
    inliers->resize(residuals.size(), false);
    double mle = 0.0;
    for (int i = 0; i < residuals.size(); i++) {
      const double& r = residuals[i];
      const double& v = confidence_[i];

      double pr_inlier = inlier_dist_.eval(r)*v;
      double pr_outlier = outlier_dist_.eval(r)*(1.0 - v);

      double arglog = pr_inlier + pr_outlier;
      // If the arglog = 0 then this is a horrible model. Return the max value
      // possible and stop computation.
      if (arglog == 0.0)
        return kInfinity;

      // If our confidence of this measurement is sufficiently high, add it to
      // the inlier list.
      double confidence = pr_inlier/arglog;
      if (confidence > confidence_threshold_) {
        inliers->at(i) = true;

        mle += log(arglog);
      }
    }
    return mle;
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
  const math::Distribution& inlier_dist_;
  // Distribution of outlier data.
  const math::Distribution& outlier_dist_;
  // Confidences of each data point.
  const std::vector<double>& confidence_;
};
}  // namespace solvers
#endif  // SOLVERS_MLE_QUALITY_MEASUREMENT_H_
