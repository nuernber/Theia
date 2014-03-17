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

#ifndef THEIA_IMAGE_DESCRIPTOR_DESCRIPTOR_EXTRACTOR_H_
#define THEIA_IMAGE_DESCRIPTOR_DESCRIPTOR_EXTRACTOR_H_

#include <glog/logging.h>
#include <Eigen/Core>
#include <algorithm>
#include <vector>

#include "theia/util/util.h"
#include "theia/alignment/alignment.h"

namespace theia {
template<class T> class Image;
typedef Image<float> GrayImage;
class Keypoint;

// This interface class is meant to define all descriptor extractors. From a
// high level, there are two types of descriptors (and extractors): float and
// binary. Float descriptors are more common (e.g., SIFT) while binary
// descriptors (e.g., BRISK) have become more popular as of late. The interfact
// defines methods for both types, though only one type may be implemented per
// class.
//
// NOTE: VectorXb is a boolean-type vector and is defined as a typedef in
// alingment.h
class DescriptorExtractor {
 public:
  DescriptorExtractor() {}
  virtual ~DescriptorExtractor() {}

  // This method should be called before using any of the descriptor
  // extractors. The constuctor should only be used for get/set methods. Any
  // computationally expensive or non-trivial operations should go in the
  // Initialize method.
  virtual bool Initialize() { return true; }

  // Computes a floatdescriptor at a single keypoint.
  virtual bool ComputeDescriptor(const GrayImage& image,
                                 const Keypoint& keypoint,
                                 Eigen::Vector2d* feature_position,
                                 Eigen::VectorXf* descriptor) = 0;

  // Computes a binary descriptor at a single keypoint.
  virtual bool ComputeDescriptor(const GrayImage& image,
                                 const Keypoint& keypoint,
                                 Eigen::Vector2d* feature_position,
                                 Eigen::VectorXf* descriptor) = 0;

  // Compute the descriptors for multiple keypoints in a given image. This
  // method will populate teh feature_position and descriptors vectors
  // accordingly. Only the features that could extract a valid descriptor are in
  // these containers, so it is not guaranteed that all keypoints will extract a
  // features (e.g., keypoints near the edge of an image). Returns true on
  // success, false on failure.
  virtual bool ComputeDescriptors(
      const GrayImage& image,
      const std::vector<Keypoint>& keypoints,
      std::vector<Eigen::Vector2d>* feature_position,
      std::vector<Eigen::VectorXf>* descriptors);

  // Same as above, but for binary descriptors.
  virtual bool ComputeDescriptors(
      const GrayImage& image,
      const std::vector<Keypoint>& keypoints,
      std::vector<Eigen::Vector2d>* feature_position,
      std::vector<Eigen::VectorXb>* descriptors);

 private:
  DISALLOW_COPY_AND_ASSIGN(DescriptorExtractor);
};



// Class for float descriptor extractors. All classes that extract float
// descriptors should be derived from this class.
class FloatDescriptorExtractor {
  FloatDescriptorExtractor() {}
  virtual ~FloatDescriptorExtractor() {}

  virtual bool Initialize() { return true; }

  // Computes a floatdescriptor at a single keypoint.
  virtual bool ComputeDescriptor(const GrayImage& image,
                                 const Keypoint& keypoint,
                                 Eigen::Vector2d* feature_position,
                                 Eigen::VectorXf* descriptor) = 0;

  // Computes a binary descriptor at a single keypoint.
  virtual bool ComputeDescriptor(const GrayImage& image,
                                 const Keypoint& keypoint,
                                 Eigen::Vector2d* feature_position,
                                 Eigen::VectorXf* descriptor) {
#pragma message("YOU ARE ATTEMPTING TO EXTRACT A BINARY DESCRIPTOR WITH A " \
                "FLOAT DESCRIPTOR EXTRACTOR");
  }

  // Compute the descriptors for multiple keypoints in a given image.
  virtual bool ComputeDescriptors(
      const GrayImage& image,
      const std::vector<Keypoint*>& keypoints,
      std::vector<Eigen::Vector2d>* feature_position,
      std::vector<Eigen::VectorXf>* descriptors);

  // Same as above, but for binary descriptors.
  virtual bool ComputeDescriptors(
      const GrayImage& image,
      const std::vector<Keypoint*>& keypoints,
      std::vector<Eigen::Vector2d>* feature_position,
      std::vector<Eigen::VectorXb>* descriptors) {
#pragma message("YOU ARE ATTEMPTING TO EXTRACT A BINARY DESCRIPTOR WITH A " \
                "FLOAT DESCRIPTOR EXTRACTOR");
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(FloatDescriptorExtractor);
};

// Class for binary descriptor extractors. All classes that extract binary
// descriptors should be derived from this class.
class BinaryDescriptorExtractor {
  BinaryDescriptorExtractor() {}
  virtual ~BinaryDescriptorExtractor() {}

  virtual bool Initialize() { return true; }

  // Computes a floatdescriptor at a single keypoint.
  virtual bool ComputeDescriptor(const GrayImage& image,
                                 const Keypoint& keypoint,
                                 Eigen::Vector2d* feature_position,
                                 Eigen::VectorXf* descriptor) {
#pragma message("YOU ARE ATTEMPTING TO EXTRACT A FLOAT DESCRIPTOR WITH A " \
                "BINARY DESCRIPTOR EXTRACTOR");
  }

  // Computes a binary descriptor at a single keypoint.
  virtual bool ComputeDescriptor(const GrayImage& image,
                                 const Keypoint& keypoint,
                                 Eigen::Vector2d* feature_position,
                                 Eigen::VectorXf* descriptor) = 0;


  // Compute the descriptors for multiple keypoints in a given image.
  virtual bool ComputeDescriptors(
      const GrayImage& image,
      const std::vector<Keypoint*>& keypoints,
      std::vector<Eigen::Vector2d>* feature_position,
      std::vector<Eigen::VectorXf>* descriptors) {
#pragma message("YOU ARE ATTEMPTING TO EXTRACT A FLOAT DESCRIPTOR WITH A " \
                "BINARY DESCRIPTOR EXTRACTOR");
  }
  // Same as above, but for binary descriptors.
  virtual bool ComputeDescriptors(
      const GrayImage& image,
      const std::vector<Keypoint*>& keypoints,
      std::vector<Eigen::Vector2d>* feature_position,
      std::vector<Eigen::VectorXb>* descriptors);

 private:
  DISALLOW_COPY_AND_ASSIGN(BinaryDescriptorExtractor);
};

}  // namespace theia

#endif  // THEIA_IMAGE_DESCRIPTOR_DESCRIPTOR_EXTRACTOR_H_
