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

#ifndef IMAGE_IMAGE_CANVAS_H_
#define IMAGE_IMAGE_CANVAS_H_

#include "image/image_canvas.h"

#include <string>
#include <vector>
#include "image/image.h"

namespace theia {
class Keypoint;

// This class allows for drawing on top of one or many images for visualization
// without modifying the underlying image. This is useful for feature detection,
// descriptor extraction, matching, etc. Add images to the canvas using
// AddImage, then draw on the canvas appropriately. There are two versions of
// all functions: one where you can specify drawing coordinates relative to the
// image at image_index, and another where you can specify the raw canvas
// coordinates. The likely use case of the latter is when you have an image
// canvas with only one image.
//
// NOTE: ImageCanvas objects are always an rgb image underneath so that pixels
// drawn onto the canvas can be color, even if the image is not. Again, this can
// be useful for visualization and matching.
class ImageCanvas {
 public:
  ImageCanvas() {}
  ~ImageCanvas() {}

  // Add an image to the canvas such that all the images that have been added
  // are now side-by-side on the canvas. This is useful for feature matching.
  int AddImage(const GrayImage& image);
  int AddImage(const RGBImage& image);

  // Draw a circle in the image at image_index.
  void DrawCircle(int image_index, int x, int y, int radius,
                  const RGBPixel& color);
  // Draw a circle onto the canvas.
  void DrawCircle(int x, int y, int radius, const RGBPixel& color);

  // Draw a line in the image at image_index.
  void DrawLine(int image_index, int x1, int y1, int x2, int y2,
                const RGBPixel& color);
  // Draw a line onto the image canvas.
  void DrawLine(int x1, int y1, int x2, int y2, const RGBPixel& color);

  // Draw a cross in the image at image_index.
  void DrawCross(int image_index, int x, int y, int length,
                 const RGBPixel& color);
  // Draw a cross onto the image canvas.
  void DrawCross(int x, int y, int length, const RGBPixel& color);

  // Draw a box in the image at image_index.
  void DrawBox(int image_index, int x, int y, int x_length, int y_length,
               const RGBPixel& color);
  // Draw a box onto the image canvas.
  void DrawBox(int x, int y, int x_length, int y_length,
               const RGBPixel& color);

  // Draw keypoint in the image at image_index.
  void DrawFeature(int image_index, const Keypoint& keypoint,
                   const RGBPixel& color, double scale = 10.0);
  // Draw the keypoint onto the image canvas.
  void DrawFeature(const Keypoint& keypoint, const RGBPixel& color,
                   double scale = 10.0);

  // Draw keypoint in the image at image_index.
  void DrawFeatures(int image_index, const std::vector<Keypoint*>& keypoints,
                    const std::vector<RGBPixel>& colors, double scale = 10.0);
  void DrawFeatures(int image_index, const std::vector<Keypoint*>& keypoints,
                    const RGBPixel& color, double scale = 10.0);
  // Draw the keypoint onto the image canvas.
  void DrawFeatures(const std::vector<Keypoint*>& keypoints,
                    const std::vector<RGBPixel>& colors, double scale = 10.0);
  void DrawFeatures(const std::vector<Keypoint*>& keypoints,
                    const RGBPixel& color, double scale = 10.0);

  // Draw matching keypoints in the image by drawing a line from keypoints1[i]
  // to keypoints2[i].
  void DrawMatchedFeatures(int image_index1,
                           const std::vector<Keypoint*>& keypoints1,
                           int image_index2,
                           const std::vector<Keypoint*>& keypoints2);

  // Write the image canvas to a file.
  void Write(const std::string& output_name);

 private:
  // The local copy of the canvas. This can be comprised of image(s), shape(s),
  // and more.
  CVD::Image<RGBPixel> image_;

  // Contains the starting x coordinate of the image corresponding to the
  // index. This makes it easy to draw points relative to a particular image
  // when there are multiple images on the canvas.
  std::vector<int> pixel_offsets_;
};
}  // namespace theia
#endif  // IMAGE_IMAGE_CANVAS_H_
