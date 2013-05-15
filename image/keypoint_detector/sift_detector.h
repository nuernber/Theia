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

#ifndef IMAGE_KEYPOINT_DETECTOR_SIFT_DETECTOR_H_
#define IMAGE_KEYPOINT_DETECTOR_SIFT_DETECTOR_H_

extern "C" {
  #include <vl/sift.h>
}

#include <vector>

#include "image/keypoint_detector/keypoint_detector.h"

namespace theia {
template<class T> class Image;
typedef Image<float> GrayImage;
class Keypoint;

// SIFT detector as originally proposed by David Lowe. This relies on the open
// source software VLFeat (www.vlfeat.org) to detect keypoints.
class SiftDetector : public KeypointDetector {
 public:
  // Set the feature score threshold and indicate whether nonmax suppression
  // should be used to reduce the number of features. A good value for the
  // threshold is usually 20.
  SiftDetector(int num_octaves, int num_levels, int first_octave);
  SiftDetector() : SiftDetector(-1, 3, 0) {}
  ~SiftDetector();

  // Given an image, detect keypoints using the sift descriptor.
  bool DetectKeypoints(const GrayImage& image,
                       std::vector<Keypoint*>* keypoints);
 private:
  VlSiftFilt* sift_filter_;
  int num_octaves_;
  int num_levels_;
  int first_octave_;
};
}  // namespace theia
#endif  // IMAGE_KEYPOINT_DETECTOR_SIFT_DETECTOR_H_
