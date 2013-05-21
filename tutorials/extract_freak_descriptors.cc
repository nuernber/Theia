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

DEFINE_string(input_image, "image.png",
              "Image to extract PATCH keypoints and descriptors from.");
DEFINE_string(output_dir, "output.png", "Name of output image file.");

using theia::AgastDetector;
using theia::GrayImage;
using theia::ImageCanvas;
using theia::Keypoint;
using theia::FreakDescriptor;
using theia::FreakDescriptorExtractor;

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  GrayImage image(FLAGS_input_image);

  // Detect keypoints.
  VLOG(0) << "detecting keypoints";
  AgastDetector keypoint_detector;
  std::vector<Keypoint*> agast_keypoints;
  keypoint_detector.DetectKeypoints(image, &agast_keypoints);
  VLOG(0) << "detected " << agast_keypoints.size() << " keypoints.";

  // Extract descriptors. 
  VLOG(0) << "extracting descriptors.";
  FreakDescriptorExtractor freak_extractor(false, false, 1);
  std::vector<FreakDescriptor*> freak_descriptors;
  freak_extractor.Initialize();
  freak_extractor.ComputeDescriptors(image, agast_keypoints, &freak_descriptors);
  int num_descriptors;
  for (int i = 0; i < freak_descriptors.size(); i++)
    if (freak_descriptors[i] != nullptr)
      num_descriptors++;
      
  VLOG(0) << "extracted " << num_descriptors << " descriptors.";
 
  // Get an image canvas to draw the features on.
  ImageCanvas image_canvas;
  image_canvas.AddImage(image);
  image_canvas.DrawFeatures(agast_keypoints, theia::RGBPixel(1.0, 0, 0));
  image_canvas.DrawFeatures(freak_descriptors, theia::RGBPixel(0, 0, 1.0));
  image_canvas.Write(FLAGS_output_dir +
                     std::string("/agast_keypoints.png"));
}
