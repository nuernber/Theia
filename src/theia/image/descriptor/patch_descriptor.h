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

#ifndef THEIA_IMAGE_DESCRIPTOR_PATCH_DESCRIPTOR_H_
#define THEIA_IMAGE_DESCRIPTOR_PATCH_DESCRIPTOR_H_

#include <vector>

#include "theia/image/descriptor/descriptor.h"
#include "theia/image/descriptor/descriptor_extractor.h"
#include "theia/image/image.h"
#include "theia/util/util.h"

namespace theia {
class Keypoint;

// R should be the number of rows, c should be the number of columns in the
// patch e.g. a 7x7 patch would be PatchDescriptor<7,7>.
class PatchDescriptor : public FloatDescriptor {
 public:
  PatchDescriptor(int rows, int cols)
      : FloatDescriptor(rows * cols, DescriptorType::PATCH) {}
};

class PatchDescriptorExtractor : public DescriptorExtractor {
 public:
  PatchDescriptorExtractor(int patch_rows, int patch_cols)
      : patch_rows_(patch_rows), patch_cols_(patch_cols) {}
  ~PatchDescriptorExtractor() {}

  // Computes a descriptor at a single keypoint.
  Descriptor* ComputeDescriptor(const GrayImage& image,
                                const Keypoint& keypoint);

 private:
  int patch_rows_;
  int patch_cols_;

  DISALLOW_COPY_AND_ASSIGN(PatchDescriptorExtractor);
};

}  // namespace theia

#endif  // THEIA_IMAGE_DESCRIPTOR_PATCH_DESCRIPTOR_H_
