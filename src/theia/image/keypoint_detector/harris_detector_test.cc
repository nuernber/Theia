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

#include <cvd/harris_corner.h>
#include <cvd/image_convert.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <string>
#include <utility>

#include "gtest/gtest.h"
#include "theia/image/image.h"
#include "theia/image/keypoint_detector/harris_detector.h"
#include "theia/image/keypoint_detector/keypoint.h"
#include "theia/util/random.h"

DEFINE_string(test_img, "image/keypoint_detector/img1.png",
              "Name of test image file.");

namespace theia {
std::string img_filename = THEIA_TEST_DATA_DIR + std::string("/") +
                           FLAGS_test_img;

TEST(HarrisDetector, BasicTest) {
  GrayImage input_img(img_filename);
  const CVD::Image<float>& cvd_img = input_img.GetCVDImage();

  // Get the keypoints our way.
  HarrisDetector harris_detector(500);
  std::vector<Keypoint*> harris_keypoints;
  harris_detector.DetectKeypoints(input_img, &harris_keypoints);

  // Get the keypoints through CVD.
  std::vector<std::pair<float, CVD::ImageRef> > cvd_corners;
  CVD::Image<float> xx(cvd_img.size()), xy(cvd_img.size()), yy(cvd_img.size());
  CVD::harrislike_corner_detect<CVD::Harris::HarrisScore,
                                CVD::Harris::PairInserter>(cvd_img,
                                                           cvd_corners,
                                                           500,
                                                           1.0,
                                                           3.0,
                                                           xx, xy, yy);

  // Compare to ensure that they are equal!
  ASSERT_EQ(harris_keypoints.size(), 500);
  ASSERT_EQ(harris_keypoints.size(), cvd_corners.size());
  for (int i = 0; i < harris_keypoints.size(); i++) {
    ASSERT_EQ(harris_keypoints[i]->keypoint_type(), Keypoint::HARRIS);
    ASSERT_EQ(harris_keypoints[i]->x(), cvd_corners[i].second.x);
    ASSERT_EQ(harris_keypoints[i]->y(), cvd_corners[i].second.y);
    ASSERT_EQ(harris_keypoints[i]->strength(), cvd_corners[i].first);
  }
}

}  // namespace theia
