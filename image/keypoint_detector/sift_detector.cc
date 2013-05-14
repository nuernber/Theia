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

#include "image/keypoint_detector/sift_detector.h"

extern "C" {
#include <vl/sift.h>
}

#include <vector>

#include "image/image.h"
#include "image/keypoint_detector/keypoint.h"

namespace theia {
SiftDetector::~SiftDetector() {
  if (sift_filter_ != nullptr)
    vl_sift_delete(sift_filter_);
}

bool SiftDetector::DetectKeypoints(const GrayImage& image,
                                   std::vector<Keypoint*>* keypoints) {
  // NOTE: This is a bug in vlfeat! These functions are not set by default.
  vl_set_alloc_func(malloc, realloc, calloc, free);

  // If the filter has been set, but is not usable for the input image (i.e. the
  // width and height are different) then we must make a new filter. Adding this
  // statement will save the function from regenerating the filter for
  // successive calls with images of the same size (e.g. a video sequence).
  VLOG(0) << "checking if filter is null";
  if (sift_filter_ != nullptr && (sift_filter_->width != image.Cols() ||
                               sift_filter_->height != image.Rows())) {
    VLOG(0) << "deleting pointer you should not delete!";
    vl_sift_delete(sift_filter_);
  }
  // If the filter has not been set (or was deleted in the previous if), then we
  // need to create a new filter.
  VLOG(0) << "potentially creating new filter: "
          << "\n\timage width, height = " << image.Cols() << ", " << image.Rows()
          << "\n\toctaves = " << num_octaves_
          << "\n\tlevels = " << num_levels_
          << "\n\tfirst octaves = " << first_octave_;
  if (sift_filter_ == nullptr)
    sift_filter_ = vl_sift_new(image.Cols(), image.Rows(),
                               num_octaves_,
                               num_levels_,
                               first_octave_);
  
  // The VLFeat functions take in a non-const image pointer so that it can
  // calculate gaussian pyramids. Obviously, we do not want to break our const
  // input, so the best solution (for now) is to copy the image.
  VLOG(0) << "cloning image.";
  GrayImage mutable_image = image.Clone();

  // Calculate the first octave to process.
  VLOG(0) << "processing first filter";
  int vl_status = vl_sift_process_first_octave(sift_filter_,
                                               mutable_image.GetData());
  // Process octaves until you can't anymore.
  while (vl_status != VL_ERR_EOF) {
    // Detect the keypoints.
    VLOG(0) << "detecting keypoints";
    vl_sift_detect(sift_filter_);
    // Get the keypoints.
    VLOG(0) << "getting keypoints";
    const VlSiftKeypoint* vl_keypoints = vl_sift_get_keypoints(sift_filter_);
    int num_keypoints = vl_sift_get_nkeypoints(sift_filter_);
    VLOG(0) << "got " << num_keypoints << " keypoints!";

    for (int i = 0; i < num_keypoints; i++) {
      // Calculate (up to 4) orientations of the keypoint.
      double angles[4];
      int num_angles = vl_sift_calc_keypoint_orientations(sift_filter_,
                                                          angles,
                                                          &vl_keypoints[i]);
      for (int j = 0; j < num_angles; j++) {
        Keypoint* keypoint = new Keypoint(vl_keypoints[i].x,
                                          vl_keypoints[i].y,
                                          Keypoint::SIFT);
        keypoint->set_scale(vl_keypoints[i].sigma);
        keypoint->set_orientation(angles[j]);
        // TODO(cmsweeney): there is no preallocation of the vector. This could
        // have bad performance!
        keypoints->push_back(keypoint);
      }
    }
    VLOG(0) << "processing next octave";
    // Attempt to process the next octave.
    vl_status = vl_sift_process_next_octave(sift_filter_);
  }
  return true;
}
}  // namespace theia
