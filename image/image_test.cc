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

#include <chrono>
#include <math.h>
#include <random>
#include <stdio.h>
#include <string>
#include <vector>

#include <cvd/image.h>

#include <gflags/gflags.h>
#include "gtest/gtest.h"
#include "image/image.h"
#include "test/test_utils.h"

DEFINE_string(test_img, "test1.jpg", "Directory where the test images "
              "are located");

namespace theia {
namespace {
std::string img_filename =
    THEIA_TEST_DATA_DIR + std::string("/") + FLAGS_test_img;

#define ASSERT_RGB_IMG_EQ(cvd_img, theia_img, rows, cols) \
  for (int i = 0; i < rows; i++) { \
    for (int j = 0; j < cols; j++) { \
      ASSERT_EQ(cvd_img[i][j].red, theia_img[i][j].red); \
      ASSERT_EQ(cvd_img[i][j].green, theia_img[i][j].green); \
      ASSERT_EQ(cvd_img[i][j].blue, theia_img[i][j].blue); \
    } \
  } 
}

#define ASSERT_GRAY_IMG_EQ(cvd_img, theia_img, rows, cols) \
  for (int i = 0; i < rows; i++) \
    for (int j = 0; j < cols; j++) \
      ASSERT_EQ(cvd_img[i][j], theia_img[i][j]); \

// Test that inputting the old fashioned way is the same as through our class.
TEST(Image, RGBInput) {
  CVD::Image<CVD::Rgb<float> > cvd_img = CVD::img_load(img_filename);

  RGBImage theia_img(img_filename);

  int rows = cvd_img.size().y;
  int cols = cvd_img.size().x;

  // Assert each pixel value is exactly the same!
  ASSERT_RGB_IMG_EQ(cvd_img, theia_img, rows, cols);
}

// Test that width and height methods work.
TEST(Image, RGBWidthHeight) {
  CVD::Image<CVD::Rgb<float> > cvd_img = CVD::img_load(img_filename);
  RGBImage theia_img(img_filename);

  int true_height = cvd_img.size().y;
  int true_width = cvd_img.size().x;

  ASSERT_EQ(theia_img.Width(), true_width);
  ASSERT_EQ(theia_img.Height(), true_height);
}

TEST(Image, RGBGetImage) {
  CVD::Image<CVD::Rgb<float> > cvd_img = CVD::img_load(img_filename);
  RGBImage theia_img(img_filename);
  CVD::Image<CVD::Rgb<float> > theia_cvd_img = theia_img.GetCVDImage();
  
  int rows = cvd_img.size().y;
  int cols = cvd_img.size().x;

  // Assert each pixel value is exactly the same!
  ASSERT_RGB_IMG_EQ(cvd_img, theia_cvd_img, rows, cols);
}

TEST(Image, RGBClone) {
  RGBImage theia_img(img_filename);
  RGBImage theia2_img = theia_img.Clone();

  int rows = theia_img.Height();
  int cols = theia_img.Width();

  ASSERT_RGB_IMG_EQ(theia_img, theia2_img, rows, cols);
}

// Test that inputting the old fashioned way is the same as through our class.
TEST(Image, GrayInput) {
  CVD::Image<float> cvd_img = CVD::img_load(img_filename);

  GrayImage theia_img(img_filename);

  int rows = cvd_img.size().y;
  int cols = cvd_img.size().x;

  // Assert each pixel value is exactly the same!
  ASSERT_GRAY_IMG_EQ(cvd_img, theia_img, rows, cols);
}

// Test that width and height methods work.
TEST(Image, GrayWidthHeight) {
  CVD::Image<float> cvd_img = CVD::img_load(img_filename);
  GrayImage theia_img(img_filename);

  int true_height = cvd_img.size().y;
  int true_width = cvd_img.size().x;

  ASSERT_EQ(theia_img.Width(), true_width);
  ASSERT_EQ(theia_img.Height(), true_height);
}

TEST(Image, GrayGetImage) {
  CVD::Image<float> cvd_img = CVD::img_load(img_filename);
  GrayImage theia_img(img_filename);
  CVD::Image<float> theia_cvd_img = theia_img.GetCVDImage();
  
  int rows = cvd_img.size().y;
  int cols = cvd_img.size().x;

  // Assert each pixel value is exactly the same!
  ASSERT_GRAY_IMG_EQ(cvd_img, theia_cvd_img, rows, cols);
}

TEST(Image, GrayClone) {
  GrayImage theia_img(img_filename);
  GrayImage theia2_img = theia_img.Clone();

  int rows = theia_img.Height();
  int cols = theia_img.Width();

  ASSERT_GRAY_IMG_EQ(theia_img, theia2_img, rows, cols);
}

}  // namespace theia
