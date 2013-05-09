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

#ifndef IMAGE_DESCRIPTOR_PATCH_DESCRIPTOR_H_
#define IMAGE_DESCRIPTOR_PATCH_DESCRIPTOR_H_

#include <vector>

#include "image/descriptor/descriptor.h"
#ifndef THEIA_NO_PROTOCOL_BUFFERS
#include "image/descriptor/descriptor.pb.h"
#endif
#include "image/descriptor/descriptor_extractor.h"
#include "image/image.h"
#include "image/keypoint_detector/keypoint.h"

namespace theia {
template<class D> class PatchDescriptorExtractor;

// R should be the number of rows, c should be the number of columns in the
// patch e.g. a 7x7 patch would be PatchDescriptor<7,7>.
template<int R, int C>
class PatchDescriptor : public Descriptor<float, R*C> {
 public:
  PatchDescriptor() : patch_rows_(R), patch_cols_(C) {}
 private:
  friend class PatchDescriptorExtractor<PatchDescriptor<R, C> >;
  int patch_rows_;
  int patch_cols_;
};

template<class D>
class PatchDescriptorExtractor : public DescriptorExtractor<D> {
 public:
  PatchDescriptorExtractor() {}
  ~PatchDescriptorExtractor() {}

  // Computes a descriptor at a single keypoint.
  bool ComputeDescriptor(
      const GrayImage& image,
      const Keypoint* keypoint,
      D* descriptor);

  // Methods to load/store descriptors in protocol buffers.
#ifndef THEIA_NO_PROTOCOL_BUFFERS
  bool ProtoToDescriptor(const DescriptorsProto& proto,
                         std::vector<D*>* descriptors) const;

  bool DescriptorToProto(const std::vector< D*>& descriptors,
                         DescriptorsProto* proto) const;
#endif  // THEIA_NO_PROTOCOL_BUFFERS
};

// ----------------- IMPLEMENTATION ----------------- //

// Computes a descriptor at a single keypoint.
template<class D>
bool PatchDescriptorExtractor<D>::ComputeDescriptor(
    const GrayImage& image,
    const Keypoint* keypoint,
    D* descriptor) {
  int patch_r_begin = keypoint->y - descriptor->patch_rows_/2;
  int patch_c_begin = keypoint->x - descriptor->patch_cols_/2;
  int patch_r_end = patch_r_begin + descriptor->patch_rows_;
  int patch_c_end = patch_c_begin + descriptor->patch_cols_;
  
  // Check the bounds to make sure the patch is within the image.
  CHECK_GE(patch_r_begin, 0) << "Extracting patch outside of image.";
  CHECK_GE(patch_c_begin, 0) << "Extracting patch outside of image.";
  CHECK_LE(patch_r_end, image.Rows()) << "Extracting patch outside of image.";
  CHECK_LE(patch_c_end, image.Cols()) << "Extracting patch outside of image.";
  int i = 0;
  for (int r = patch_r_begin; r < patch_r_end; r++) {
    for (int c = patch_c_begin; c < patch_c_end; c++) {
      (*descriptor)[i++] = image[r][c];
    }
  }
}

// Methods to load/store descriptors in protocol buffers.
#ifndef THEIA_NO_PROTOCOL_BUFFERS
template<class D>
bool PatchDescriptorExtractor<D>::ProtoToDescriptor(
    const DescriptorsProto& proto,
    std::vector<D*>* descriptors) const {
  descriptors->reserve(proto.feature_descriptor_size());
  for (const DescriptorProto& proto_descriptor: proto.feature_descriptor()) {
    D* descriptor = new D;
    CHECK_EQ(proto_descriptor.descriptor_type(), DescriptorProto::PATCH)
        << "Descriptor in proto is not a patch descriptor.";
    CHECK_EQ(proto_descriptor.float_descriptor_size(), descriptor->Dimensions())
        << "Dimension mismatch in the proto and descriptors.";
    // Get float array.
    for (int i = 0; i < descriptor->Dimensions(); i++)
      (*descriptor)[i] = proto_descriptor.float_descriptor(i);
    descriptors->push_back(descriptor);
  }
  return true;
}

template<class D>
bool PatchDescriptorExtractor<D>::DescriptorToProto(
    const std::vector<D*>& descriptors,
    DescriptorsProto* proto) const {
  for (const D* descriptor : descriptors) {
    DescriptorProto* descriptor_proto = proto->add_feature_descriptor();
    // Add the float array to the proto.
    for (int i = 0; i < descriptor->Dimensions(); i++)
      descriptor_proto->add_float_descriptor((*descriptor)[i]);
    // Set the proto type to patch.
    descriptor_proto->set_descriptor_type(DescriptorProto::PATCH);
  }
  return true;
}
#endif  // THEIA_NO_PROTOCOL_BUFFERS
}  // namespace theia

#endif  // IMAGE_DESCRIPTOR_PATCH_DESCRIPTOR_H_
