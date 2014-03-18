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

#include "theia/image/descriptor/descriptor_extractor.h"

#include <Eigen/Core>

#include "theia/alignment/alignment.h"
<<<<<<< HEAD
=======
#include "theia/image/keypoint_detector/keypoint.h"
>>>>>>> d742cce

namespace theia {
// Compute the descriptor for multiple keypoints in a given image.
bool DescriptorExtractor::ComputeDescriptors(
    const GrayImage& image,
<<<<<<< HEAD
    const std::vector<Keypoint*>& keypoints,
=======
    const std::vector<Keypoint>& keypoints,
>>>>>>> d742cce
    std::vector<Eigen::Vector2d>* feature_positions,
    std::vector<Eigen::VectorXf>* descriptors) {
  VLOG(0) << "calling base version... bad!";
  descriptors->reserve(keypoints.size());
<<<<<<< HEAD
  for (const Keypoint* img_keypoint : keypoints) {
    Eigen::VectorXf descriptor;
    Eigen::Vector2d feature_position;
    if (ComputeDescriptor(image, *img_keypoint, &feature_position,
=======
  for (const Keypoint& img_keypoint : keypoints) {
    Eigen::VectorXf descriptor;
    Eigen::Vector2d feature_position;
    if (ComputeDescriptor(image, img_keypoint, &feature_position,
>>>>>>> d742cce
                          &descriptor)) {
      feature_positions->push_back(feature_position);
      descriptors->push_back(descriptor);
    } else {
      return false;
    }
  }
  return true;
}

bool DescriptorExtractor::ComputeDescriptors(
    const GrayImage& image,
<<<<<<< HEAD
    const std::vector<Keypoint*>& keypoints,
    std::vector<Eigen::Vector2d>* feature_positions,
    std::vector<Eigen::VectorXb>* descriptors) {
  VLOG(0) << "calling base version... bad!";
  descriptors->reserve(keypoints.size());
  for (const Keypoint* img_keypoint : keypoints) {
    Eigen::VectorXb descriptor;
    Eigen::Vector2d feature_position;
    if (ComputeDescriptor(image, *img_keypoint, &feature_position,
=======
    const std::vector<Keypoint>& keypoints,
    std::vector<Eigen::Vector2d>* feature_positions,
    std::vector<Eigen::BinaryVectorX>* descriptors) {
  VLOG(0) << "calling base version... bad!";
  descriptors->reserve(keypoints.size());
  for (const Keypoint& img_keypoint : keypoints) {
    Eigen::BinaryVectorX descriptor;
    Eigen::Vector2d feature_position;
    if (ComputeDescriptor(image, img_keypoint, &feature_position,
>>>>>>> d742cce
                          &descriptor)) {
      feature_positions->push_back(feature_position);
      descriptors->push_back(descriptor);
    } else {
      return false;
    }
  }
  return true;
}

}  // namespace theia
