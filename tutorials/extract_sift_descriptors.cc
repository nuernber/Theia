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

#include "theia/image/image.h"
#include "theia/image/image_canvas.h"
#include "theia/image/descriptor/sift_descriptor.h"
#include "theia/image/keypoint_detector/keypoint.h"
#include "theia/image/keypoint_detector/sift_detector.h"

DEFINE_string(input_image, "image.png",
              "Image to extract SIFT keypoints and descriptors from.");
DEFINE_string(output_dir, ".", "Name of output image file.");

using theia::GrayImage;
using theia::ImageCanvas;
using theia::Keypoint;
using theia::Descriptor;
using theia::SiftDescriptorExtractor;
using theia::SiftDetector;

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  GrayImage image(FLAGS_input_image);

  // Detect keypoints.
  VLOG(0) << "detecting keypoints";
  SiftDetector keypoint_detector;
  std::vector<Keypoint> sift_keypoints;
  keypoint_detector.DetectKeypoints(image, &sift_keypoints);
  VLOG(0) << "detected " << sift_keypoints.size() << " keypoints.";

  // Extract descriptors.
  VLOG(0) << "extracting descriptors.";
  SiftDescriptorExtractor sift_extractor;
  std::vector<Descriptor*> sift_descriptors;
  sift_extractor.ComputeDescriptors(image, sift_keypoints, &sift_descriptors);
  VLOG(0) << "extracted " << sift_descriptors.size() << " descriptors.";

  // Get an image canvas to draw the features on.
  ImageCanvas image_canvas;
  image_canvas.AddImage(image);
  image_canvas.DrawFeatures(sift_keypoints, theia::RGBPixel(0, 0, 1.0));
  image_canvas.Write(FLAGS_output_dir +
                     std::string("/detect_then_extract.png"));

  // Detect and Extract in one shot!
  VLOG(0) << "detect and extract together.";
  std::vector<Descriptor*> sift_features;
  SiftDescriptorExtractor feature_extractor;
  feature_extractor.DetectAndExtractDescriptors(image, &sift_features);

  ImageCanvas image_canvas2;
  image_canvas2.AddImage(image);
  image_canvas2.DrawFeatures(sift_features, theia::RGBPixel(0, 0, 1));
  image_canvas2.Write(FLAGS_output_dir +
                      std::string("/detect_and_extract.png"));
}
