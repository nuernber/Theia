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

#ifndef IMAGE_IMAGE_H_
#define IMAGE_IMAGE_H_

#include <glog/logging.h>
#include <cvd/image.h>
#include <cvd/image_convert.h>
#include <cvd/image_io.h>
#include <string>

namespace theia {

// A general Image class for all images used in Theia. This is basically a
// wrapper for the libCVD Image class. Note the typedefs after the class
// definition as well.
template <typename T>
class Image;

template <typename T>
class SubImage {
 public:
  explicit SubImage(CVD::SubImage<T> sub) : sub_image_(sub) {}
  ~SubImage() {}

  // Rows, cols.
  int Rows() { return sub_image_.size().y; }
  int Cols() { return sub_image_.size().x; }

  // Accessors.
  T* operator[] (int row) { return sub_image_[row]; }
  const T* operator[] (int row) const { return sub_image_[row]; }

  // Get the Sub Image.
  CVD::SubImage<T>& GetCVDImage() { return sub_image_; }
  const CVD::SubImage<T>& GetCVDImage() const { return sub_image_; }

  // Return the current SubImage as an Image i.e. copy the data so that the
  // returned Image object owns the copy of the data.
  Image<T> CopyAsImage() {
    Image<T> new_img;
    new_img.copy_from(sub_image_);
    return new_img;
  }

 protected:
  // Only for internal use!
  SubImage() : sub_image_(NULL, CVD::ImageRef(0, 0), 0) {}
  CVD::SubImage<T> sub_image_;
};

template <typename T>
class Image : public SubImage<T> {
 public:
  Image() {}
  ~Image() {}

  // Copy from a CVD image. Only copies the pointer. See DeepCopy for a full
  // copy.
  explicit Image(CVD::Image<T> copy_img) {
    image_ = copy_img;
    sub_image_ = image_;
  }
  
  // Read from file.
  explicit Image(std::string filename) {
    image_ = CVD::img_load(filename);
    image_.make_unique();
    sub_image_ = image_;
  }

  // Clones via deep copy.
  Image<T> Clone() { return Image(image_.copy_from_me()); }

  // Deep copies the input to this.
  void DeepCopy(Image<T> copy_img) {
    image_ = copy_img.copy_from_me();
    sub_image_ = image_;
  }

  // Read image from file.
  void Read(std::string filename) { *this = Image(filename); }

  // Write image to file.
  void Write(std::string filename) { CVD::img_save(image_, filename); }

  // Get the full Image.
  CVD::Image<T>& GetCVDImage() { return image_; }
  const CVD::Image<T>& GetCVDImage() const { return image_; }

  // Get pixel data as array.
  T* Data() { return image_.data(); }
  const T* Data() const { return image_.data(); }

  // Extract SubImages from the Image. Note that this does not give ownership of
  // the data to the subimage, the original Image class still owns it.
  SubImage<T> GetSubImage(int row, int col, int num_rows, int num_cols) {
    return SubImage<T>(image_.sub_image(CVD::ImageRef(col, row),
                                        CVD::ImageRef(num_cols, num_rows)));
  }
  const Image<T> GetSubImage(int row, int col,
                             int num_rows, int num_cols) const {
    return SubImage<T>(row, col, num_rows, num_cols);
  }

 protected:
  CVD::Image<T> image_;
  using SubImage<T>::sub_image_;
};

typedef CVD::Rgb<float> RGBPixel;
typedef float Pixel;

// RGB Image shorthand.
typedef SubImage<RGBPixel> RGBSubImage;
typedef Image<RGBPixel> RGBImage;

// Grayscale Image shorthand.
typedef SubImage<Pixel> GraySubImage;
typedef Image<Pixel> GrayImage;

}  // namespace theia

#endif  // IMAGE_IMAGE_H_
