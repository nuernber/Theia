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
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "experiments/sift_matches.pb.h"
#include "image/descriptor/sift_descriptor.h"
#include "image/image.h"
#include "image/keypoint_detector/keypoint.h"
#include "image/keypoint_detector/sift_detector.h"
#include "vision/matching/brute_force_matcher.h"
#include "vision/matching/distance.h"
#include "vision/matching/image_matcher.h"

DEFINE_string(input_filename, "image_files.txt", "Path to a file containing "
              "file path's to the input image set.");
DEFINE_string(output_dir, "", "Output directory to write the sift matches.");
DEFINE_int32(knn, 200, "Number (k) of nearest neighbors to find.");

using namespace theia;

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // Read the input file to get image file paths.
  std::vector<std::string> image_filenames;
  std::ifstream in_stream;
  std::string line;
  in_stream.open(FLAGS_input_filename);
  while (!in_stream.eof()) {
    in_stream >> line;
    image_filenames.push_back(line);
  }
  in_stream.close();

  // Detect all keypoints and extract descriptors for every image.
  std::vector<GrayImage> images;
  std::vector<std::vector<SiftDescriptor*> > descriptors;
  images.reserve(image_filenames.size());
  descriptors.reserve(image_filenames.size());
  for (const std::string& image_filename : image_filenames) {
    GrayImage image(image_filename);

    // Detect keypoints.
    VLOG(0) << "detecting keypoints";
    SiftDetector sift_detector;
    std::vector<Keypoint*> keypoints;
    sift_detector.DetectKeypoints(image, &keypoints);
    VLOG(0) << "detected " << keypoints.size()
            << " keypoints in left image.";

    // Extract descriptors.
    VLOG(0) << "extracting descriptors.";
    SiftDescriptorExtractor sift_extractor;
    sift_extractor.Initialize();
    std::vector<SiftDescriptor*> pruned_descriptors;
    sift_extractor.ComputeDescriptorsPruned(image,
                                            keypoints,
                                            &pruned_descriptors);
    VLOG(0) << "pruned descriptors size = " << pruned_descriptors.size();

    images.push_back(image);
    descriptors.push_back(pruned_descriptors);
  }

  // Output format is as follows:
  //
  // num_camera_pairs
  // num_features_for_camera_pair_1   (we'll call this m1)
  // image_0_id image_1_id feature_1_id dist_1 dist_2 ... dist_k
  // ...
  // image_0_id image_1_id feature_m1_id dist_1 dist_2 ... dist_k
  // num_features_for_camera_pair_2   (we'll call this m2)
  // image_0_id image_2_id feature_1_id dist_1 dist_2 ... dist_k
  // ...
  //
  // We do this for each camera pair until all num_camera_pairs have been
  // written.

  // For each image pair, match descriptors.
  //output_file.open(FLAGS_output_filename);
  ImagePairMatchesProto image_pair_matches;
  for (int i = 0; i < images.size(); i++) {
    // Build a matcher that matches to image i.
    BruteForceMatcher<SiftDescriptor, theia::L2<float> > brute_force_matcher;
    brute_force_matcher.Build(descriptors[i]);
    for (int j = 0; j < images.size(); j++) {
      VLOG(0) << "matching image " << i << " to image " << j;

      // Find knn of features from image j to image i.
      std::vector<std::vector<int> > knn_index;
      std::vector<std::vector<float> > knn_dist;
      brute_force_matcher.KNearestNeighbors(descriptors[j],
                                            FLAGS_knn,
                                            &knn_index,
                                            &knn_dist);
      // Create an image pair output.
      ImagePairMatchProto* image_pair_match =
          image_pair_matches.add_image_pair_match();
      image_pair_match->set_image1_id(j);
      image_pair_match->set_image2_id(i);

      // For each descriptor.
      for (int k = 0; k < knn_dist.size(); k++) {
        // Add a feature match to this image pair.
        FeatureMatchProto* feature_match =
            image_pair_match->add_feature_match();
        feature_match->set_feature_id(j);
        feature_match->set_scale(descriptors[j][k]->scale());

        // For each of the knn for this descriptor.
        for (int knn_num = 0; knn_num < FLAGS_knn; knn_num++) {
          // Add each knn to this feature match.
          FeatureKnnProto* knn_match = feature_match->add_knn_match();
          knn_match->set_feature_id(knn_index[k][knn_num]);
          knn_match->set_distance(knn_dist[k][knn_num]);
          knn_match->set_scale(descriptors[i][knn_index[k][knn_num]]->scale());
        }
      }
    }
  }

  std::fstream out(FLAGS_output_dir + "sift_matches.pb",
                   std::ios::out | std::ios::binary | std::ios::trunc);
  image_pair_matches.SerializeToOstream(&out);
  out.close();
}
