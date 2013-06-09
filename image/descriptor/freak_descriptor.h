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

#ifndef IMAGE_DESCRIPTOR_FREAK_DESCRIPTOR_H_
#define IMAGE_DESCRIPTOR_FREAK_DESCRIPTOR_H_

#include <vector>

#include "image/descriptor/descriptor.h"
#include "image/descriptor/descriptor_extractor.h"
#include "util/util.h"

namespace theia {
class DescriptorsProto;
template<class T> class Image;
typedef Image<float> GrayImage;
class Keypoint;

// The FREAK descriptor as described in "FREAK: Fast Retina Keypoint" by Alahi
// et. al (CVPR 2012).
class FreakDescriptor : public BinaryDescriptor<512> {
 public:
  FreakDescriptor() : BinaryDescriptor(DescriptorType::FREAK) {}
};

class FreakDescriptorExtractor : public DescriptorExtractor<FreakDescriptor> {
 public:
  // Params:
  //  orientation_normalized: Enable orientation normalization.
  //  scale_normalized: Enable scale normalization.
  //  num_octaves: Number of octaves covered by the keypoints.
  FreakDescriptorExtractor(bool orientation_normalized,
                           bool scale_normalized,
                           int num_octaves)
      : orientation_normalized_(orientation_normalized),
        scale_normalized_(scale_normalized),
        num_octaves_(num_octaves),
        pattern_scale_(22.0) {}

  FreakDescriptorExtractor() : FreakDescriptorExtractor(true, true, 4) {}
  ~FreakDescriptorExtractor() {}

  // Initializes the sampling patterns and local variables.
  bool Initialize();

  // Computes a descriptor at a single keypoint.
  bool ComputeDescriptor(const GrayImage& image,
                         const Keypoint& keypoint,
                         FreakDescriptor* descriptor);

  // Compute multiple descriptors for keypoints from a single image. Note this
  // may return null for some of the descriptors if they cannot be computed!
  // Typically this only happens when it is too close to the border.
  bool ComputeDescriptors(const GrayImage& image,
                          const std::vector<Keypoint*>& keypoints,
                          std::vector<FreakDescriptor*>* descriptors);
  // Methods to load/store descriptors in protocol buffers. Each derived class
  // should implement these methods (if desired) and load/store all appropriate
  // fields in the protocol buffer. This is kind of a sucky paradigm since these
  // methods act more like static methods, but it is the best way to make sure
  // these methods are paired to the descriptors.
#ifndef THEIA_NO_PROTOCOL_BUFFERS
  bool ProtoToDescriptor(const DescriptorsProto& proto,
                         std::vector<FreakDescriptor*>* descriptors) const;

  bool DescriptorToProto(
      const std::vector<FreakDescriptor*>& descriptors,
      DescriptorsProto* proto) const;
#endif

  enum {
    kNumScales_ = 64,
    kNumPairs_ = 512,
    kNumOrientationPairs_ = 45
  };

 private:
  DISALLOW_COPY_AND_ASSIGN(FreakDescriptorExtractor);
  
  float MeanIntensity(const GrayImage& image, const GrayImage& integral,
                      const float kp_x,
                      const float kp_y,
                      const unsigned int scale,
                      const unsigned int rot,
                      const unsigned int point) const;

  bool orientation_normalized_;
  bool scale_normalized_;
  bool num_octaves_;
  const float pattern_scale_;

  std::vector<int> selected_pairs_;

  struct PatternPoint {
    // x coordinate relative to center
    float x;
    // x coordinate relative to center
    float y;
    // Gaussian smoothing sigma
    float sigma;
  };

  struct DescriptionPair {
    // index of the first point
    unsigned char i;
    // index of the second point
    unsigned char j;
  };

  struct OrientationPair {
    // index of the first point
    unsigned char i;
    // index of the second point
    unsigned char j;
    // dx/(norm_sq))*4096
    int weight_dx;
    // dy/(norm_sq))*4096
    int weight_dy;
  };


  // look-up table for the pattern points (position+sigma of all points at all
  // scales and orientation)
  std::vector<PatternPoint> pattern_lookup_;
  // size of the pattern at a specific scale (used to check if a point is within
  // image boundaries)
  int pattern_sizes_[kNumScales_];
  DescriptionPair description_pairs_[kNumPairs_];
  OrientationPair orientation_pairs_[kNumOrientationPairs_];

  inline void SetOrientationPair(int arr, unsigned char i, unsigned char j) {
    orientation_pairs_[arr].i = i;
    orientation_pairs_[arr].j = j;
  }
};
}  // namespace theia
#endif  // IMAGE_DESCRIPTOR_FREAK_DESCRIPTOR_H_
