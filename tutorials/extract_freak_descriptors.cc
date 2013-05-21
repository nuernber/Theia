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

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <string>
#include <vector>

#include "image/image.h"
#include "image/image_canvas.h"
#include "image/descriptor/freak_descriptor.h"
#include "image/keypoint_detector/keypoint.h"
#include "image/keypoint_detector/agast_detector.h"

DEFINE_string(img_input_dir, "input", "Directory of two input images.");
DEFINE_string(img_output_dir, "output", "Name of output image file.");

using theia::AgastDetector;
using theia::GrayImage;
using theia::ImageCanvas;
using theia::Keypoint;
using theia::FreakDescriptor;
using theia::FreakDescriptorExtractor;

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  GrayImage image_left(FLAGS_img_input_dir + std::string("/img1.png"));
  GrayImage image_right(FLAGS_img_input_dir + std::string("/img2.png"));

  // Detect keypoints.
  VLOG(0) << "detecting keypoints";
  AgastDetector keypoint_detector;
  std::vector<Keypoint*> left_keypoints;
  keypoint_detector.DetectKeypoints(image_left, &left_keypoints);
  VLOG(0) << "detected " << left_keypoints.size()
          << " keypoints in left image.";
  std::vector<Keypoint*> right_keypoints;
  keypoint_detector.DetectKeypoints(image_left, &right_keypoints);
  VLOG(0) << "detected " << right_keypoints.size()
          << " keypoints in right image.";

  // Extract descriptors.
  VLOG(0) << "extracting descriptors.";
  FreakDescriptorExtractor freak_extractor(false, false, 1);
  std::vector<FreakDescriptor*> left_descriptors;
  std::vector<FreakDescriptor*> right_descriptors;
  freak_extractor.Initialize();
  freak_extractor.ComputeDescriptors(image_left,
                                     left_keypoints,
                                     &left_descriptors);
  freak_extractor.ComputeDescriptors(image_right,
                                     right_keypoints,
                                     &right_descriptors);

  int num_descriptors;
  for (int i = 0; i < left_descriptors.size(); i++)
    if (left_descriptors[i] != nullptr)
      num_descriptors++;

  VLOG(0) << "extracted " << num_descriptors << " descriptors.";
  
  std::vector<FreakDescriptor*> pruned_descriptors;
  freak_extractor.ComputeDescriptorsPruned(image_left,
                                           left_keypoints,
                                           &pruned_descriptors);
  VLOG(0) << "pruned descriptors size = " << pruned_descriptors.size();
  
  // Match descriptors!

  // Get an image canvas to draw the features on.
  ImageCanvas image_canvas;
  image_canvas.AddImage(image_left);
  image_canvas.DrawFeatures(left_descriptors, theia::RGBPixel(1.0, 0, 0));
  image_canvas.AddImage(image_right);
  image_canvas.DrawFeatures(right_descriptors, theia::RGBPixel(0, 0, 1.0));
  image_canvas.Write(FLAGS_img_output_dir +
                     std::string("/agast_keypoints.png"));
}
