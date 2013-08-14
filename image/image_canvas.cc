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

#include "image/image_canvas.h"

#include <cvd/draw.h>
#include <glog/logging.h>

#include <cmath>
#include <string>
#include <vector>

#include "image/image.h"
#include "image/keypoint_detector/keypoint.h"

#define _USE_MATH_DEFINES

namespace theia {
// Add an image to the canvas such that all the images that have been added
// are now side-by-side on the canvas. This is useful for feature matching.
int ImageCanvas::AddImage(const GrayImage& image) {
  RGBImage rgb_img(image.ConvertTo<RGBPixel>());
  return AddImage(rgb_img);
}

int ImageCanvas::AddImage(const RGBImage& image) {
  if (pixel_offsets_.size() == 0) {
    image_ = image.GetCVDImage().copy_from_me();
    pixel_offsets_.push_back(0);
  } else {
    pixel_offsets_.push_back(image_.size().x);
    CVD::Image<RGBPixel> image_copy = image_.copy_from_me();
    CVD::joinImages(image_copy, image.GetCVDImage(), image_);
  }
  return pixel_offsets_.size() - 1;
}

// Draw a circle in the image at image_index.
void ImageCanvas::DrawCircle(int image_index, int x, int y, int radius,
                             const RGBPixel& color) {
  CHECK_GT(pixel_offsets_.size(), image_index)
      << "Trying to draw in an image index that does not exist!";
  DrawCircle(pixel_offsets_[image_index] + x, y, radius, color);
}
// Draw a circle onto the canvas.
void ImageCanvas::DrawCircle(int x, int y, int radius, const RGBPixel& color) {
  std::vector<CVD::ImageRef> circle_pixels = CVD::getCircle(radius);
  CVD::drawShape(image_, CVD::ImageRef(x, y), circle_pixels, color);
}

// Draw a line in the image at image_index.
void ImageCanvas::DrawLine(int image_index, int x1, int y1, int x2, int y2,
                           const RGBPixel& color) {
  CHECK_GT(pixel_offsets_.size(), image_index)
      << "Trying to draw in an image index that does not exist!";
  DrawLine(pixel_offsets_[image_index] + x1, y1,
           pixel_offsets_[image_index] + x2, y2, color);
}

// Draw a line onto the image canvas.
void ImageCanvas::DrawLine(int x1, int y1, int x2, int y2,
                           const RGBPixel& color) {
  CVD::drawLine(image_, x1, y1, x2, y2, color);
}

// Draw a cross in the image at image_index.
void ImageCanvas::DrawCross(int image_index, int x, int y, int length,
                            const RGBPixel& color) {
  CHECK_GT(pixel_offsets_.size(), image_index)
      << "Trying to draw in an image index that does not exist!";
  DrawCross(pixel_offsets_[image_index] + x, y, length, color);
}
// Draw a cross onto the image canvas.
void ImageCanvas::DrawCross(int x, int y, int length, const RGBPixel& color) {
  CVD::drawCross(image_, CVD::ImageRef(x, y), length, color);
}

// Draw a box in the image at image_index.
void ImageCanvas::DrawBox(int image_index,
                          int x, int y,
                          int x_length, int y_length,
                          const RGBPixel& color) {
  CHECK_GT(pixel_offsets_.size(), image_index)
      << "Trying to draw in an image index that does not exist!";
  DrawBox(pixel_offsets_[image_index] + x, y, x_length, y_length, color);
}

// Draw a box onto the image canvas.
void ImageCanvas::DrawBox(int x, int y, int x_length, int y_length,
                          const RGBPixel& color) {
  CVD::drawBox(image_, CVD::ImageRef(x, y),
               CVD::ImageRef(x + x_length, y + y_length), color);
}

void ImageCanvas::DrawFeature(int image_index,
                              int x, int y,
                              int radius,
                              double orientation,
                              const RGBPixel& color) {
  CHECK_GT(pixel_offsets_.size(), image_index)
      << "Trying to draw in an image index that does not exist!";
  // Draw circle at keypoint with size scale*strength.
  DrawCircle(image_index, x, y, radius, color);

  // Draw line in direction of the orientation if applicable.
  DrawLine(image_index,
           x, y,
           x + radius*cos(orientation), y + radius*sin(orientation),
           color);
}

// Write the image canvas to a file.
void ImageCanvas::Write(const std::string& output_name) {
  CVD::img_save(image_, output_name);
}
}  // namespace theia
