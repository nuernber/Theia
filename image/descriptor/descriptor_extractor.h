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

#ifndef IMAGE_DESCRIPTOR_DESCRIPTOR_EXTRACTOR_H_
#define IMAGE_DESCRIPTOR_DESCRIPTOR_EXTRACTOR_H_

#include <glog/logging.h>
#include <algorithm>
#include <vector>

namespace theia {
class DescriptorsProto;
template<class T, std::size_t N> class GenericDescriptor;
template<class T> class Image;
typedef Image<float> GrayImage;
class Keypoint;

// Templating this class is a little bit ugly, but it guarantees coupling
// between your extractors and descriptors. This is a high priority from a
// design perspective.
template<class D>
class DescriptorExtractor {
 public:
  DescriptorExtractor() {}
  virtual ~DescriptorExtractor() {}

  // This method should be called before using any of the descriptor
  // extractors. The constuctor should only be used for get/set methods. Any
  // computationally expensive or non-trivial operations should go in the
  // Initialize method.
  virtual bool Initialize() { return true; }

  // Computes a descriptor at a single keypoint.
  virtual bool ComputeDescriptor(const GrayImage& image,
                                 const Keypoint& keypoint,
                                 D* descriptor) = 0;

  // Compute the descriptors for multiple keypoints in a given image. This
  // method will allocate the D* pointers, but the caller owns the data and must
  // delete it. Returns false upon failure. The index of each descriptor will
  // match the index to its corresponding keypoint. Thus, the output vector may
  // contain null values for some of the descriptors (if a descriptor cannot be
  // computed at that keypoint). This can happen for various reasons, including
  // being too close to the border.
  virtual bool ComputeDescriptors(const GrayImage& image,
                                  const std::vector<Keypoint*>& keypoints,
                                  std::vector<D*>* descriptors);

  // Computes all descriptors possible and returns a vector of descriptors will
  // null descriptors removed. NOTE: this means that the indices of the
  // descriptors and keypoints are not guaranteed to match! If you want them to
  // match, use ComptuDescriptors instead.
  virtual bool ComputeDescriptorsPruned(const GrayImage& image,
                                        const std::vector<Keypoint*>& keypoints,
                                        std::vector<D*>* descriptors);

  // Methods to load/store descriptors in protocol buffers. Each derived class
  // should implement these methods (if desired) and load/store all appropriate
  // fields in the protocol buffer. This is kind of a sucky paradigm since these
  // methods act more like static methods, but it is the best way to make sure
  // these methods are paired to the descriptors.
#ifndef THEIA_NO_PROTOCOL_BUFFERS
  virtual bool ProtoToDescriptor(const DescriptorsProto& proto,
                                 std::vector<D*>* descriptors) const = 0;

  virtual bool DescriptorToProto(const std::vector<D*>& descriptors,
                                 DescriptorsProto* proto) const = 0;
#endif
};

// ------------------- IMPLEMENTATION ------------------- //
// Compute the descriptor for multiple keypoints in a given image.
template<class D>
bool DescriptorExtractor<D>::ComputeDescriptors(
    const GrayImage& image,
    const std::vector<Keypoint*>& keypoints,
    std::vector<D*>* descriptors) {
  VLOG(0) << "calling base version... bad!";
  descriptors->reserve(keypoints.size());
  for (const Keypoint* img_keypoint : keypoints) {
    D* descriptor = new D;
    ComputeDescriptor(image, *img_keypoint, descriptor);
    descriptors->push_back(descriptor);
  }
  return true;
}

template<class D>
bool DescriptorExtractor<D>::ComputeDescriptorsPruned(
    const GrayImage& image,
    const std::vector<Keypoint*>& keypoints,
    std::vector<D*>* descriptors) {
  if (ComputeDescriptors(image, keypoints, descriptors)) {
    VLOG(0) << "calling pruned version!";
    // Erase all elements of descriptors that were set to nullptr.
    descriptors->erase(std::remove_if(descriptors->begin(), descriptors->end(),
                                      [](D* x) { return x == nullptr; }),
                       descriptors->end());
    return true;
  } else {
    return false;
  }
}
}  // namespace theia

#endif  // IMAGE_DESCRIPTOR_DESCRIPTOR_EXTRACTOR_H_
