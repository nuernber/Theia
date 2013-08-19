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

#include "image/keypoint_detector/brisk_detector.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <string>

#include "gtest/gtest.h"
#include "image/image.h"
#include "image/keypoint_detector/keypoint.h"
#ifndef THEIA_NO_PROTOCOL_BUFFERS
#include "image/keypoint_detector/keypoint.pb.h"
#endif
#include "util/random.h"

DEFINE_string(test_img, "image/keypoint_detector/img1.png",
              "Name of test image file.");

namespace theia {
std::string img_filename = THEIA_TEST_DATA_DIR + std::string("/") +
                           FLAGS_test_img;

TEST(BriskDetector, Sanity) {
  GrayImage input_img(img_filename);

  // Get the keypoints our way.
  BriskDetector brisk_detector;
  ASSERT_TRUE(brisk_detector.Initialize());
  std::vector<Keypoint*> brisk_keypoints;
  ASSERT_TRUE(brisk_detector.DetectKeypoints(input_img, &brisk_keypoints));
}

// Protocol buffer tests.
#ifndef THEIA_NO_PROTOCOL_BUFFERS
TEST(BriskDetector, ProtoTest) {
  InitRandomGenerator();
  std::vector<Keypoint*> brisk_keypoints;
  for (int i = 0; i < 100; i++) {
    Keypoint* brisk_keypoint = new Keypoint(RandDouble(0, 500),
                                            RandDouble(0, 500),
                                            Keypoint::BRISK);
    brisk_keypoint->set_strength(RandDouble(0, 100));
    brisk_keypoints.push_back(brisk_keypoint);
  }

  KeypointsProto brisk_proto;
  KeypointToProto(brisk_keypoints, &brisk_proto);

  std::vector<Keypoint*> proto_keypoints;
  ProtoToKeypoint(brisk_proto, &proto_keypoints);

  ASSERT_EQ(brisk_keypoints.size(), 100);
  ASSERT_EQ(brisk_keypoints.size(), proto_keypoints.size());
  for (int i = 0; i < brisk_keypoints.size(); i++) {
    ASSERT_EQ(brisk_keypoints[i]->keypoint_type(),
              proto_keypoints[i]->keypoint_type());
    ASSERT_EQ(brisk_keypoints[i]->x(), proto_keypoints[i]->x());
    ASSERT_EQ(brisk_keypoints[i]->y(), proto_keypoints[i]->y());
    ASSERT_EQ(brisk_keypoints[i]->strength(), proto_keypoints[i]->strength());
  }
}
#endif  // THEIA_NO_PROTOCOL_BUFFERS

}  // namespace theia
