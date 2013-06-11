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

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <time.h>
#include <string>
#include <vector>

#include "image/image.h"
#include "image/image_canvas.h"
#include "image/descriptor/sift_descriptor.h"
#include "image/keypoint_detector/keypoint.h"
#include "image/keypoint_detector/sift_detector.h"
#include "vision/matching/distance.h"
#include "vision/matching/brute_force_matcher.h"
#include "vision/matching/image_matcher.h"

DEFINE_string(img_input_dir, "input", "Directory of two input images.");
DEFINE_string(img_output_dir, "output", "Name of output image file.");
DEFINE_int32(knn, 5, "number k nearest neighbors to find.");

using theia::SiftDescriptor;
using theia::SiftDescriptorExtractor;
using theia::SiftDetector;
using theia::BruteForceMatcher;
using theia::GrayImage;
using theia::ImageCanvas;
using theia::Keypoint;

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  GrayImage left_image(FLAGS_img_input_dir + std::string("/img1.png"));
  GrayImage right_image(FLAGS_img_input_dir + std::string("/img2.png"));

  // Detect keypoints.
  VLOG(0) << "detecting keypoints";
  SiftDetector sift_detector;
  std::vector<Keypoint*> left_keypoints;
  sift_detector.DetectKeypoints(left_image, &left_keypoints);
  VLOG(0) << "detected " << left_keypoints.size()
          << " keypoints in left image.";

  VLOG(0) << "detecting keypoints";
  std::vector<Keypoint*> right_keypoints;
  sift_detector.DetectKeypoints(right_image, &right_keypoints);
  VLOG(0) << "detected " << left_keypoints.size()
          << " keypoints in left image.";

  // Extract descriptors.
  VLOG(0) << "extracting descriptors.";
  SiftDescriptorExtractor sift_extractor;
  sift_extractor.Initialize();

  std::vector<SiftDescriptor*> left_pruned_descriptors;
  sift_extractor.ComputeDescriptorsPruned(left_image,
                                           left_keypoints,
                                           &left_pruned_descriptors);
  VLOG(0) << "pruned descriptors size = " << left_pruned_descriptors.size();

  std::vector<SiftDescriptor*> right_pruned_descriptors;
  sift_extractor.ComputeDescriptorsPruned(right_image,
                                           right_keypoints,
                                           &right_pruned_descriptors);
  VLOG(0) << "pruned descriptors size = " << right_pruned_descriptors.size();

  // Match descriptors!
  // Match NN
  for (int i = 0; i < left_pruned_descriptors.size(); i++) {
    BruteForceMatcher<SiftDescriptor, theia::L2<float> > brute_force_matcher;
    brute_force_matcher.Build(right_pruned_descriptors);
    int nn_index;
    float distance;
    brute_force_matcher.NearestNeighbor(*left_pruned_descriptors[i],
                                        &nn_index,
                                        &distance);
    VLOG(0) << "Top NN: " << nn_index << " of dist " << distance;
    // Match kNN
    std::vector<int> knn_index;
    std::vector<float> knn_dist;
    brute_force_matcher.KNearestNeighbors(*left_pruned_descriptors[i],
                                          FLAGS_knn,
                                          &knn_index,
                                          &knn_dist);
    for (int j = 0; j < FLAGS_knn; j++) {
      VLOG(0) << "kNN[" << j << "]: " << knn_index[j] << " of dist "
              << knn_dist[j];
    }
    VLOG(0) << "";
  }
}
