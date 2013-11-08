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

#include "theia/image/descriptor/patch_descriptor.h"

#include <vector>

#include "theia/image/descriptor/descriptor.h"
#include "theia/image/descriptor/descriptor_extractor.h"
#include "theia/image/image.h"
#include "theia/image/keypoint_detector/keypoint.h"
#include "theia/util/util.h"

namespace theia {
// Computes a descriptor at a single keypoint.
Descriptor* PatchDescriptorExtractor::ComputeDescriptor(
    const GrayImage& image,
    const Keypoint& keypoint) {
  PatchDescriptor* descriptor = new PatchDescriptor(patch_rows_, patch_cols_);
  int patch_r_begin = keypoint.y() - patch_rows_/2;
  int patch_c_begin = keypoint.x() - patch_cols_/2;
  int patch_r_end = patch_r_begin + patch_rows_;
  int patch_c_end = patch_c_begin + patch_cols_;

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
  descriptor->SetKeypoint(keypoint);
  return descriptor;
}

}  // namespace theia
