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

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <string>
#include "gtest/gtest.h"

#include "image/image.h"
#include "image/keypoint_detector/sift_detector.h"
#include "image/descriptor/sift_descriptor.h"
#ifndef THEIA_NO_PROTOCOL_BUFFERS
#include "image/descriptor/descriptor.pb.h"
#endif

DEFINE_string(test_img, "image/descriptor/img1.png",
              "Name of test image file.");
namespace theia {
namespace {
std::string img_filename = THEIA_TEST_DATA_DIR + std::string("/") +
                           FLAGS_test_img;
}  // namespace

TEST(SiftDescriptor, Sanity) {
  GrayImage input_img(img_filename);

  // Get keypoints.
  SiftDetector fast_detector;
  std::vector<Keypoint*> sift_keypoints;
  fast_detector.DetectKeypoints(input_img, &sift_keypoints);

  // For each keypoint, extract the sift descriptors.
  SiftDescriptorExtractor sift_extractor;
  std::vector<Descriptor*> sift_descriptors;
  CHECK_NOTNULL(sift_extractor.ComputeDescriptor(input_img, *sift_keypoints[0]));
  CHECK(sift_extractor.ComputeDescriptorsPruned(input_img,
                                                sift_keypoints,
                                                &sift_descriptors));
}

#ifndef THEIA_NO_PROTOCOL_BUFFERS
TEST(SiftDescriptor, ProtoTest) {
  GrayImage input_img(img_filename);

  // Get keypoints.
  SiftDetector fast_detector;
  std::vector<Keypoint*> sift_keypoints;
  fast_detector.DetectKeypoints(input_img, &sift_keypoints);

  // For each keypoint, extract the sift descriptors.
  SiftDescriptorExtractor sift_extractor;
  std::vector<Descriptor*> sift_descriptors;
  sift_extractor.ComputeDescriptorsPruned(input_img,
                                          sift_keypoints,
                                          &sift_descriptors);

  // Convert the descriptors to a proto.
  DescriptorsProto sift_proto;
  sift_extractor.DescriptorToProto(sift_descriptors, &sift_proto);

  // Convert back to descriptors.
  std::vector<Descriptor*> proto_descriptors;
  sift_extractor.ProtoToDescriptor(sift_proto, &proto_descriptors);

  // Assert that they are equal.
  ASSERT_EQ(sift_descriptors.size(), proto_descriptors.size());
  for (int i = 0; i < sift_descriptors.size(); i++) {
    for (int j = 0; j < sift_descriptors[i]->Dimensions(); j++) {
      ASSERT_EQ(sift_descriptors[i]->FloatData()[j],
                proto_descriptors[i]->FloatData()[j]);
    }
  }
}
#endif  // THEIA_NO_PROTOCOL_BUFFERS
}  // namespace theia
