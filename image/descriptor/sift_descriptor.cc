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

#include "image/descriptor/sift_descriptor.h"

#include <glog/logging.h>
extern "C" {
#include <vl/sift.h>
}
#include <vector>

#include "image/image.h"
#include "image/descriptor/descriptor.h"
#ifndef THEIA_NO_PROTOCOL_BUFFERS
#include "image/descriptor/descriptor.pb.h"
#endif
#include "image/descriptor/descriptor_extractor.h"
#include "image/keypoint_detector/keypoint.h"

namespace theia {

SiftDescriptorExtractor::~SiftDescriptorExtractor() {
  if (sift_filter_ != NULL)
    vl_sift_delete(sift_filter_);
}

bool SiftDescriptorExtractor::ComputeDescriptor(const GrayImage& image,
                                                const Keypoint* keypoint,
                                                SiftDescriptor* descriptor) {
  // NOTE: This is a bug in vlfeat! These functions are not set by default.
  vl_set_alloc_func(malloc, realloc, calloc, free);

  CHECK(keypoint->has_scale() && keypoint->has_orientation())
      << "Keypoint must have scale and orientation to compute a SIFT "
      << "descriptor.";
  // If the filter has been set, but is not usable for the input image (i.e. the
  // width and height are different) then we must make a new filter. Adding this
  // statement will save the function from regenerating the filter for
  // successive calls with images of the same size (e.g. a video sequence).
  if (sift_filter_ != NULL && (sift_filter_->width != image.Cols() ||
                               sift_filter_->height != image.Rows()))
    vl_sift_delete(sift_filter_);

  // If the filter has not been set (or was deleted in the previous if), then we
  // need to create a new filter.
  if (sift_filter_ == NULL)
    sift_filter_ = vl_sift_new(image.Cols(), image.Rows(),
                               num_octaves_,
                               num_levels_,
                               first_octave_);

  // Create the vl sift keypoint from the one passed in.
  VlSiftKeypoint sift_keypoint;
  vl_sift_keypoint_init(sift_filter_,
                        &sift_keypoint,
                        keypoint->x(),
                        keypoint->y(),
                        keypoint->scale());

  // The VLFeat functions take in a non-const image pointer so that it can
  // calculate gaussian pyramids. Obviously, we do not want to break our const
  // input, so the best solution (for now) is to copy the image.
  GrayImage mutable_image = image.Clone();

  // Calculate the first octave to process.
  int vl_status = vl_sift_process_first_octave(sift_filter_,
                                               mutable_image.GetData());
  // Proceed through the octaves we reach the same one as the keypoint.
  while (sift_keypoint.o != sift_filter_->o_cur)
    vl_sift_process_next_octave(sift_filter_);

  if (vl_status == VL_ERR_EOF)
    return false;

  // Calculate the sift feature. Note that we are passing in a direct pointer to
  // the descriptor's underlying data.
  vl_sift_calc_keypoint_descriptor(sift_filter_,
                                   descriptor->Data(),
                                   &sift_keypoint,
                                   keypoint->orientation());
  return true;
}

bool SiftDescriptorExtractor::ComputeDescriptors(
    const GrayImage& image,
    const std::vector<Keypoint*>& keypoints,
    std::vector<SiftDescriptor*>* descriptors) {
  // NOTE: This is a bug in vlfeat! These functions are not set by default.
  vl_set_alloc_func(malloc, realloc, calloc, free);

  // If the filter has been set, but is not usable for the input image (i.e. the
  // width and height are different) then we must make a new filter. Adding this
  // statement will save the function from regenerating the filter for
  // successive calls with images of the same size (e.g. a video sequence).
  if (sift_filter_ != NULL && (sift_filter_->width != image.Cols() ||
                               sift_filter_->height != image.Rows()))
    vl_sift_delete(sift_filter_);

  // If the filter has not been set (or was deleted in the previous if), then we
  // need to create a new filter.
  if (sift_filter_ == NULL)
    sift_filter_ = vl_sift_new(image.Cols(), image.Rows(),
                               num_octaves_,
                               num_levels_,
                               first_octave_);

  // Create the vl sift keypoint from the one passed in.
  std::vector<VlSiftKeypoint> sift_keypoints(keypoints.size());
  for (int i = 0; i < keypoints.size(); i++) {
    CHECK(keypoints[i]->has_scale() && keypoints[i]->has_orientation())
        << "Keypoint must have scale and orientation to compute a SIFT "
        << "descriptor.";
    vl_sift_keypoint_init(sift_filter_,
                          &sift_keypoints[i],
                          keypoints[i]->x(),
                          keypoints[i]->y(),
                          keypoints[i]->scale());
  }
  // The VLFeat functions take in a non-const image pointer so that it can
  // calculate gaussian pyramids. Obviously, we do not want to break our const
  // input, so the best solution (for now) is to copy the image.
  GrayImage mutable_image = image.Clone();

  // Calculate the first octave to process.
  int vl_status = vl_sift_process_first_octave(sift_filter_,
                                               mutable_image.GetData());

  // Proceed through the octaves we reach the same one as the keypoint.  We
  // first resize the descriptors vector so that the keypoint indicies will be
  // properly matched to the descriptors.
  descriptors->resize(keypoints.size(), nullptr);
  while (vl_status != VL_ERR_EOF) {
    // Go through each keypoint to see if it came from this octave.
    for (int i = 0; i < sift_keypoints.size(); i++) {
      if (sift_keypoints[i].o != sift_filter_->o_cur)
        continue;
      (*descriptors)[i] = new SiftDescriptor;
      vl_sift_calc_keypoint_descriptor(sift_filter_,
                                       (*descriptors)[i]->Data(),
                                       &sift_keypoints[i],
                                       keypoints[i]->orientation());
    }
    vl_status = vl_sift_process_next_octave(sift_filter_);
  }
  return true;
}

#ifndef THEIA_NO_PROTOCOL_BUFFERS
bool SiftDescriptorExtractor::ProtoToDescriptor(
    const DescriptorsProto& proto,
    std::vector<SiftDescriptor*>* descriptors) const {

}

bool SiftDescriptorExtractor::DescriptorToProto(
    const std::vector<SiftDescriptor*>& descriptors,
    DescriptorsProto* proto) const {

}
#endif  // THEIA_NO_PROTOCOL_BUFFERS
}  // namespace theia
