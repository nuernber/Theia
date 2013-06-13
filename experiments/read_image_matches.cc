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
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/io/coded_stream.h>
#include "sift_matches.pb.h"

DEFINE_string(pb_filename, "image_matches.pb", "Path to proto output containing "
              "image match information.");


int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  std::ifstream in(FLAGS_pb_filename, std::ios::in | std::ios::binary);
  google::protobuf::io::IstreamInputStream istream_input(&in);
  // Iterate over all image pairs.
  unsigned int size;
  while (true) {
    google::protobuf::io::CodedInputStream coded_in_stream(&istream_input);

    if (coded_in_stream.ReadVarint32(&size)) {
      google::protobuf::io::CodedInputStream::Limit msg_limit =
          coded_in_stream.PushLimit(size);

      ImagePairMatchProto image_pair_match;
      if (image_pair_match.ParseFromCodedStream(&coded_in_stream)) {
        coded_in_stream.PopLimit(msg_limit);

        const int image1_id = image_pair_match.image1_id();
        const int image2_id = image_pair_match.image2_id();
        VLOG(0) << "Reading match for " << image1_id << ", " << image2_id;
        // In the current image pair, iterate over all features in image1.
        for (int j = 0; j < image_pair_match.feature_match_size(); j++) {
          const FeatureMatchProto& feature_match =
              image_pair_match.feature_match(j);
          const int feature_id = feature_match.feature_id();
          const float scale = feature_match.scale();

          // Iterate over all knn for this feature (in sorted order from best to
          // worst).
          for (int k = 0; k < feature_match.knn_match_size(); k++) {
            const FeatureKnnProto& knn_match = feature_match.knn_match(k);
            const int matched_feature_id = knn_match.feature_id();
            const float match_distance = knn_match.distance();
            const float matched_feature_scale = knn_match.scale();
          }
        }
      }
    } else {
      break;
    }

  }

  in.close();
}
