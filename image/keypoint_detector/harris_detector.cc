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

#include "image/keypoint_detector/harris_detector.h"

#include <cvd/harris_corner.h>
#include <utility>
#include <vector>

#include "image/image.h"
#include "image/keypoint_detector/keypoint.h"
#include "image/keypoint_detector/keypoint_detector.h"

namespace theia {
bool HarrisDetector::DetectKeypoints(const GrayImage& image,
                                     std::vector<Keypoint*>* keypoints) {
  std::vector<std::pair<float, CVD::ImageRef> > harris_corners;
  const CVD::Image<float>& cvd_img = image.GetCVDImage();
  CVD::Image<float> xx(cvd_img.size()), xy(cvd_img.size()), yy(cvd_img.size());
  CVD::harrislike_corner_detect<CVD::Harris::HarrisScore,
                                CVD::Harris::PairInserter>(cvd_img,
                                                           harris_corners,
                                                           num_corners_,
                                                           blur_,
                                                           sigma_,
                                                           xx, xy, yy);
  for (const std::pair<float, CVD::ImageRef>& harris_corner : harris_corners) {
    Keypoint* harris_keypoint = new Keypoint(harris_corner.second.x,
                                             harris_corner.second.y,
                                             Keypoint::HARRIS);
    harris_keypoint->set_strength(harris_corner.first);
    keypoints->push_back(harris_keypoint);
  }
}
}  // namespace theia
