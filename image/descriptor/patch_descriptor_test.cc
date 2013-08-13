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

#include "image/keypoint_detector/fast_detector.h"
#include "image/descriptor/patch_descriptor.h"
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

TEST(PatchDescriptor, CompWithSubImage) {
  GrayImage input_img(img_filename);

  // Get keypoints.
  FastDetector fast_detector(20, true, false);
  std::vector<Keypoint*> fast_keypoints;
  fast_detector.DetectKeypoints(input_img, &fast_keypoints);

  // For each keypoint, extract the patch descriptors.
  PatchDescriptorExtractor patch_extractor(7, 7);
  std::vector<Descriptor*> patch_descriptors;
  patch_extractor.ComputeDescriptors(input_img,
                                     fast_keypoints,
                                     &patch_descriptors);

  // Compare to SubImage.
  for (int i = 0; i < patch_descriptors.size(); i++) {
    GraySubImage sub_img = input_img.GetSubImage((*fast_keypoints[i]).y() - 3,
                                                 (*fast_keypoints[i]).x() - 3,
                                                 7,
                                                 7);
    int j = 0;
    for (int r = 0; r < 7; r++) {
      for (int c = 0; c < 7; c++) {
        ASSERT_EQ(patch_descriptors[i]->FloatData()[j++], sub_img[r][c]);
      }
    }
  }
}

#ifndef THEIA_NO_PROTOCOL_BUFFERS
TEST(PatchDescriptor, ProtoTest) {
  GrayImage input_img(img_filename);

  // Get keypoints.
  FastDetector fast_detector(20, true, false);
  std::vector<Keypoint*> fast_keypoints;
  fast_detector.DetectKeypoints(input_img, &fast_keypoints);

  // For each keypoint, extract the patch descriptors.
  PatchDescriptorExtractor patch_extractor(7, 7);
  std::vector<Descriptor*> patch_descriptors;
  patch_extractor.ComputeDescriptors(input_img,
                                     fast_keypoints,
                                     &patch_descriptors);

  // Convert the descriptors to a proto.
  DescriptorsProto patch_proto;
  patch_extractor.DescriptorToProto(patch_descriptors, &patch_proto);

  // Convert back to descriptors.
  std::vector<Descriptor*> proto_descriptors;
  patch_extractor.ProtoToDescriptor(patch_proto, &proto_descriptors);

  // Assert that they are equal.
  ASSERT_EQ(patch_descriptors.size(), proto_descriptors.size());
  for (int i = 0; i < patch_descriptors.size(); i++) {
    for (int j = 0; j < 49; j++) {
      ASSERT_EQ(patch_descriptors[i]->FloatData()[j],
                proto_descriptors[i]->FloatData()[j]);
    }
  }
}
#endif  // THEIA_NO_PROTOCOL_BUFFERS
}  // namespace theia
