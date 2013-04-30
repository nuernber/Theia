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

#ifndef IMAGE_KEYPOINT_DETECTOR_FAST_DETECTOR_H_
#define IMAGE_KEYPOINT_DETECTOR_FAST_DETECTOR_H_

#include "image/keypoint_detector/keypoint.h"
#include "image/keypoint_detector/keypoint_detector.h"

namespace theia {
class GrayImage;
class KeypointsProto;

// Inherits x, y from keypoint.
struct FastKeypoint : public Keypoint {
  double strength;
};

// Detect features as described in Machine Learning for High Speed Corner
// Detection" by Rosten and Drummand (ECCV 2006).
class FastDetector : public KeypointDetector {
 public:
  // Set the feature score threshold and indicate whether nonmax suppression
  // should be used to reduce the number of features. A good value for the
  // threshold is usually 20.
  FastDetector(int threshold, bool nonmax_suppression, bool score)
      : threshold_(threshold),
        nonmax_suppression_(nonmax_suppression),
        score_(score) {}
  ~FastDetector() {}

  bool DetectKeypoints(const GrayImage& image,
                       std::vector<Keypoint*>* keypoints) = 0;

  // Methods to load/store keypoints in protocol buffers.
#ifndef THEIA_NO_PROTOCOL_BUFFERS
  bool LoadFromProto(const KeypointsProto& proto,
                     std::vector<Keypoint*>* keypoints) = 0;
  bool ToProto(KeypointsProto* proto) = 0;
#endif

 private:
  // Threshold for the minimum corner score allowed.
  int threshold_;

  // True if you want to perform nonmaximum suppresion.
  bool nonmax_suppression_;

  // True if you want the scores of the features.
  bool score_;
};
}  // namespace theia

#endif  // IMAGE_KEYPOINT_DETECTOR_FAST_DETECTOR_H_
