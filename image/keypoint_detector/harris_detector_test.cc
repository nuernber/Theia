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
#include <cvd/image_convert.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "gtest/gtest.h"

#include <string>
#include <utility>

#include "image/image.h"
#ifndef THEIA_NO_PROTOCOL_BUFFERS
#include "image/keypoint_detector/keypoint.pb.h"
#endif
#include "test/test_utils.h"

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
    HarrisKeypoint* harris_keypoint =
        static_cast<HarrisKeypoint*>(harris_keypoints[i]);
    ASSERT_EQ(harris_keypoint->x, cvd_corners[i].second.x);
    ASSERT_EQ(harris_keypoint->y, cvd_corners[i].second.y);
    ASSERT_EQ(harris_keypoint->strength, cvd_corners[i].first);
  }
}

// Protocol buffer tests.
#ifndef THEIA_NO_PROTOCOL_BUFFERS
TEST(HarrisDetector, ProtoTest) {
  test::InitRandomGenerator();
  std::vector<Keypoint*> harris_keypoints;
  for (int i = 0; i < 100; i++) {
    HarrisKeypoint* harris_keypoint = new HarrisKeypoint;
    harris_keypoint->x = test::RandDouble(0, 500);
    harris_keypoint->y = test::RandDouble(0, 500);
    harris_keypoint->strength = test::RandDouble(0, 100);
    harris_keypoints.push_back(harris_keypoint);
  }

  KeypointsProto harris_proto;
  HarrisDetector harris_detector;
  harris_detector.KeypointToProto(harris_keypoints, &harris_proto);

  std::vector<Keypoint*> proto_keypoints;
  harris_detector.ProtoToKeypoint(harris_proto, &proto_keypoints);

  ASSERT_EQ(harris_keypoints.size(), 100);
  ASSERT_EQ(harris_keypoints.size(), proto_keypoints.size());
  for (int i = 0; i < harris_keypoints.size(); i++) {
    HarrisKeypoint* harris_keypoint =
        static_cast<HarrisKeypoint*>(harris_keypoints[i]);
    HarrisKeypoint* proto_keypoint =
        static_cast<HarrisKeypoint*>(proto_keypoints[i]);
    ASSERT_EQ(harris_keypoint->x, proto_keypoint->x);
    ASSERT_EQ(harris_keypoint->y, proto_keypoint->y);
    ASSERT_EQ(harris_keypoint->strength, proto_keypoint->strength);
  }
}
#endif  // THEIA_NO_PROTOCOL_BUFFERS

}  // namespace theia
