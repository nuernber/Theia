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

#ifndef THEIA_SOLVERS_INLIER_SUPPORT_H_
#define THEIA_SOLVERS_INLIER_SUPPORT_H_

#include <vector>

#include "theia/solvers/quality_measurement.h"

namespace theia {
// Assess quality of data by whether each error residual is less than an error
// threshold. If it is below the threshold, it is considered an inlier.
class InlierSupport : public QualityMeasurement {
 public:
  InlierSupport(double error_threshold, int min_num_inliers)
      : error_thresh_(error_threshold),
        min_num_inliers_(static_cast<double>(min_num_inliers)) {}

  ~InlierSupport() {}

  // Count the number of inliers in the data;
  double Calculate(const std::vector<double>& residuals,
                   std::vector<bool>* inliers) {
    inliers->resize(residuals.size(), false);
    double num_inliers = 0.0;
    for (int i = 0; i < residuals.size(); i++) {
      if (residuals[i] < error_thresh_) {
        num_inliers += 1.0;
        inliers->at(i) = true;
      }
    }
    return num_inliers;
  }

  // Return true if quality1 > quality2 i.e. there are more inliers in quality1.
  bool Compare(const double quality1, const double quality2) {
    return quality1 > quality2;
  }

  // Return true if the number of inliers is greater than our terminating
  // criterion.
  bool SufficientlyHighQuality(const double quality) {
    return quality > min_num_inliers_;
  }

 private:
  // Threshold for determining if a data point is an inlier.
  double error_thresh_;

  // Model is high quality when the number of inliers is greater than this
  // number.
  double min_num_inliers_;
};

}  // namespace theia

#endif  // THEIA_SOLVERS_INLIER_SUPPORT_H_
