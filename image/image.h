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

#include <cvd/image.h>
#include <cvd/image_io.h>
#include <string>

namespace theia {

// A general Image class for all images used in Theia. This is basically a
// wrapper for the libCVD Image class. Note the typedefs at the bottom.
template <typename T>
class Image {
 public:
  Image() {}
  ~Image() {}
  
  // Read from file.
  explicit Image(std::string filename) { image_ = CVD::img_load(filename); }

  // Copy from a CVD image. Only copies the pointer. See DeepCopy for a full
  // copy.
  Image(CVD::Image<T> copy_img) { image_ = copy_img; }

  // Copy from Image. Only increases reference count.
  Image(const Image<T>& copy_img) { *this = copy_img; };

  // Clones via deep copy.
  Image<T> Clone() { return Image(image_.copy_from_me()); }
  
  // Deep copies the input to this.
  void DeepCopy(Image<T> copy_img) { image_ = copy_img.copy_from_me(); }
  
  // Write image to file.
  void Write(std::string filename) { CVD::img_save(image_, filename); }
  
  // Width, height, depth.
  int Width() { return image_.size().x; }
  int Height() { return image_.size().y; }
  
  // accessors.
  T* operator[] (int row) { return image_[row]; }
  const T* operator[] (int row) const { return image_[row]; }
  
  // GetImage.
  CVD::Image<T>& GetCVDImage() { return image_; }
  const CVD::Image<T>& GetCVDImage() const { return image_; }

  // Get pixel data as array.
  T* Data() { return image_.data(); }
  const T* Data() const { return image_.data(); }
  
 protected:
  CVD::Image<T> image_;

};

// RGB Image shorthand.
typedef Image<CVD::Rgb<float> > RGBImage;

// Grayscale Image shorthand.
typedef Image<float> GrayImage;

}  // namespace theia

#endif  // IMAGE_IMAGE_H_
