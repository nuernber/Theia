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

#include "image/keypoint_detector/fast_detector.h"

#include <cvd/fast_corner.h>
#include <cvd/image_convert.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <string>

#include "gtest/gtest.h"
#include "image/image.h"
#include "image/keypoint_detector/keypoint.h"
#ifndef THEIA_NO_PROTOCOL_BUFFERS
#include "image/keypoint_detector/keypoint.pb.h"
#endif
#include "test/test_utils.h"

DEFINE_string(test_img, "image/keypoint_detector/img1.png",
              "Name of test image file.");

namespace theia {
std::string img_filename = THEIA_TEST_DATA_DIR + std::string("/") +
                           FLAGS_test_img;

TEST(FastDetector, BasicTest) {
  GrayImage input_img(img_filename);
  CVD::Image<CVD::byte> cvd_img = CVD::convert_image(input_img.GetCVDImage());

  // Get the keypoints our way.
  FastDetector fast_detector(20, false, false);
  std::vector<Keypoint*> fast_keypoints;
  fast_detector.DetectKeypoints(input_img, &fast_keypoints);

  // Get the keypoints through CVD.
  std::vector<CVD::ImageRef> cvd_corners;
  CVD::fast_corner_detect_9(cvd_img, cvd_corners, 20);

  // Compare to ensure that they are equal!
  ASSERT_EQ(fast_keypoints.size(), cvd_corners.size());
  for (int i = 0; i < fast_keypoints.size(); i++) {
    ASSERT_EQ(fast_keypoints[i]->keypoint_type(), Keypoint::FAST);
    ASSERT_EQ(fast_keypoints[i]->x(), cvd_corners[i].x);
    ASSERT_EQ(fast_keypoints[i]->y(), cvd_corners[i].y);
    ASSERT_FALSE(fast_keypoints[i]->has_strength());
  }
}

TEST(FastDetector, NonmaxSuppression) {
  GrayImage input_img(img_filename);
  CVD::Image<CVD::byte> cvd_img = CVD::convert_image(input_img.GetCVDImage());

  // Get the keypoints our way.
  FastDetector fast_detector(20, true, false);
  std::vector<Keypoint*> fast_keypoints;
  fast_detector.DetectKeypoints(input_img, &fast_keypoints);

  // Get the keypoints through CVD.
  std::vector<CVD::ImageRef> cvd_corners;
  CVD::fast_corner_detect_9_nonmax(cvd_img, cvd_corners, 20);

  // Compare to ensure that they are equal!
  ASSERT_EQ(fast_keypoints.size(), cvd_corners.size());
  for (int i = 0; i < fast_keypoints.size(); i++) {
    ASSERT_EQ(fast_keypoints[i]->keypoint_type(), Keypoint::FAST);
    ASSERT_EQ(fast_keypoints[i]->x(), cvd_corners[i].x);
    ASSERT_EQ(fast_keypoints[i]->y(), cvd_corners[i].y);
    ASSERT_FALSE(fast_keypoints[i]->has_strength());
  }
}

TEST(FastDetector, Score) {
  GrayImage input_img(img_filename);
  CVD::Image<CVD::byte> cvd_img = CVD::convert_image(input_img.GetCVDImage());

  // Get the keypoints our way.
  FastDetector fast_detector(20, true, true);
  std::vector<Keypoint*> fast_keypoints;
  fast_detector.DetectKeypoints(input_img, &fast_keypoints);

  // Get the keypoints through CVD.
  std::vector<CVD::ImageRef> cvd_corners;
  CVD::fast_corner_detect_9_nonmax(cvd_img, cvd_corners, 20);
  std::vector<int> cvd_scores;
  CVD::fast_corner_score_9(cvd_img, cvd_corners, 20, cvd_scores);

  // Compare to ensure that they are equal!
  ASSERT_EQ(fast_keypoints.size(), cvd_corners.size());
  for (int i = 0; i < fast_keypoints.size(); i++) {
    ASSERT_EQ(fast_keypoints[i]->keypoint_type(), Keypoint::FAST);
    ASSERT_EQ(fast_keypoints[i]->x(), cvd_corners[i].x);
    ASSERT_EQ(fast_keypoints[i]->y(), cvd_corners[i].y);
    ASSERT_EQ(fast_keypoints[i]->strength(), cvd_scores[i]);
  }
}

// Protocol buffer tests.
#ifndef THEIA_NO_PROTOCOL_BUFFERS
TEST(FastDetector, ProtoTest) {
  test::InitRandomGenerator();
  std::vector<Keypoint*> fast_keypoints;
  for (int i = 0; i < 100; i++) {
    Keypoint* fast_keypoint = new Keypoint(test::RandDouble(0, 500),
                                           test::RandDouble(0, 500),
                                           Keypoint::FAST);
    fast_keypoint->set_strength(test::RandDouble(0, 100));
    fast_keypoints.push_back(fast_keypoint);
  }

  KeypointsProto fast_proto;
  KeypointToProto(fast_keypoints, &fast_proto);

  std::vector<Keypoint*> proto_keypoints;
  ProtoToKeypoint(fast_proto, &proto_keypoints);

  ASSERT_EQ(fast_keypoints.size(), 100);
  ASSERT_EQ(fast_keypoints.size(), proto_keypoints.size());
  for (int i = 0; i < fast_keypoints.size(); i++) {
    ASSERT_EQ(fast_keypoints[i]->keypoint_type(),
              proto_keypoints[i]->keypoint_type());
    ASSERT_EQ(fast_keypoints[i]->x(), proto_keypoints[i]->x());
    ASSERT_EQ(fast_keypoints[i]->y(), proto_keypoints[i]->y());
    ASSERT_EQ(fast_keypoints[i]->strength(), proto_keypoints[i]->strength());
  }
}
#endif  // THEIA_NO_PROTOCOL_BUFFERS

}  // namespace theia
