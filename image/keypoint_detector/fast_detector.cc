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
// ARE DISCLAIMED. IN NO EVENT SHALL CHRIS SWEENEY BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#include "image/keypoint_detector/fast_detector.h"

#include <cvd/fast_corner.h>
#include <cvd/image_convert.h>

#include <vector>

#include "image/image.h"
#include "image/keypoint_detector/keypoint.h"
#ifndef THEIA_NO_PROTOCOL_BUFFERS
#include "image/keypoint_detector/keypoint.pb.h"
#endif

namespace theia {
bool FastDetector::DetectKeypoints(const GrayImage& image,
                                   std::vector<Keypoint*>* keypoints) {
  CVD::Image<CVD::byte> cvd_img = CVD::convert_image(image.GetCVDImage());

  std::vector<CVD::ImageRef> fast_corners;
  if (nonmax_suppression_) {
    CVD::fast_corner_detect_9_nonmax(cvd_img, fast_corners, threshold_);
  } else {
    CVD::fast_corner_detect_9(cvd_img, fast_corners, threshold_);
  }

  std::vector<int> fast_scores;
  // If we want the scores returned, calculate them here (otherwise, leave the
  // scores all at 0).
  if (score_) {
    CVD::fast_corner_score_9(cvd_img, fast_corners, threshold_, fast_scores);
  } else {
    fast_scores.resize(fast_corners.size());
  }

  keypoints->reserve(fast_corners.size());
  for (int i = 0; i < fast_corners.size(); i++) {
    FastKeypoint* fast_keypoint = new FastKeypoint;
    fast_keypoint->x = static_cast<double>(fast_corners[i].x);
    fast_keypoint->y = static_cast<double>(fast_corners[i].y);
    fast_keypoint->strength = static_cast<double>(fast_scores[i]);
    keypoints->push_back(fast_keypoint);
  }

  return true;
}

#ifndef THEIA_NO_PROTOCOL_BUFFERS
bool FastDetector::ProtoToKeypoint(const KeypointsProto& proto,
                                   std::vector<Keypoint*>* keypoints) const {
  for (const KeypointProto& proto_keypoint : proto.keypoint()) {
    FastKeypoint* fast_keypoint = new FastKeypoint;
    fast_keypoint->x = proto_keypoint.location().x();
    fast_keypoint->y = proto_keypoint.location().y();
    // Strength does not have to be set in the proto. It will return the default
    // value (0) if it is not set.
    fast_keypoint->strength = proto_keypoint.strength();
    keypoints->push_back(fast_keypoint);
  }
  return true;
}

bool FastDetector::KeypointToProto(const std::vector<Keypoint*>& keypoints,
                                   KeypointsProto* proto) const {
  for (const Keypoint* keypoint : keypoints) {
    const FastKeypoint* fast_keypoint =
        static_cast<const FastKeypoint*>(keypoint);
    KeypointProto* keypoint_proto = proto->add_keypoint();
    KeypointProto_Location* keypoint_location =
        keypoint_proto->mutable_location();
    keypoint_location->set_x(fast_keypoint->x);
    keypoint_location->set_y(fast_keypoint->y);
    keypoint_proto->set_strength(fast_keypoint->strength);
    keypoint_proto->set_keypoint_detector(KeypointProto::FAST);
  }
}
#endif

}  // namespace theia
