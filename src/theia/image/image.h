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

#ifndef THEIA_IMAGE_IMAGE_H_
#define THEIA_IMAGE_IMAGE_H_

#define _USE_MATH_DEFINES

#include <cvd/convolution.h>
#include <cvd/image.h>
#include <cvd/image_convert.h>
#include <cvd/image_io.h>
#include <cvd/integral_image.h>
#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace theia {

// A general Image class for all images used in Theia. This is basically a
// wrapper for the libCVD Image class.
template <typename T> class Image;
template <typename T> class SubImage;

typedef CVD::Rgb<float> RGBPixel;

// RGB Image shorthand.
typedef SubImage<RGBPixel> RGBSubImage;
typedef Image<RGBPixel> RGBImage;

// Grayscale Image shorthand.
typedef SubImage<float> GraySubImage;
typedef Image<float> GrayImage;

template <typename T> class SubImage {
 public:
  explicit SubImage(CVD::SubImage<T> sub) : sub_image_(sub) {}
  ~SubImage() {}

  // Rows, cols.
  int Rows() const { return sub_image_.size().y; }
  int Cols() const { return sub_image_.size().x; }

  // Accessors.
  T* operator[](int row) { return sub_image_[row]; }
  const T* operator[](int row) const { return sub_image_[row]; }

  // Fill with a scalar.
  void Fill(const T& val) { sub_image_.fill(val); }

  // Get the Sub Image.
  CVD::SubImage<T>& GetCVDImage() { return sub_image_; }
  const CVD::SubImage<T>& GetCVDImage() const { return sub_image_; }

  // Get a pointer to the data. Note this only returns a pointer to the first
  // position in the sub-image, so it is still possible to access pixels outside
  // of the subimage, since SubImages are pointers to data managed elsewhere.
  T* Data() { return sub_image_.data(); }

  // Return the current SubImage as an Image i.e. copy the data so that the
  // returned Image object owns the copy of the data.
  Image<T> CopyAsImage() const {
    Image<T> new_img;
    new_img.image_.copy_from(sub_image_);
    return new_img;
  }

 protected:
  // Only for internal use!
  SubImage() : sub_image_(NULL, CVD::ImageRef(0, 0), 0) {}
  CVD::SubImage<T> sub_image_;
};

template <typename T> class Image : public SubImage<T> {
 public:
  Image() {}
  ~Image() {}

  // Copy from a CVD image. Only copies the pointer by default. Set to true if
  // you want this object to create its own (deep) copy of the data.
  explicit Image(const CVD::Image<T> copy_img, bool deep_copy = false);
  // Read from file.
  explicit Image(std::string filename);
  // Initialize empty image.
  Image(int rows, int cols);
  Image(int rows, int cols, T val);

  static Image<T> Zeros(int rows, int cols);
  // Convert to other image types.
  // NOTE: the way to call this would be theia_image.ConvertTo<RGBPixel>()
  // because you need explicit template arguments for the conversion.
  template <typename C> Image<C> ConvertTo() const;

  // Clones via deep copy.
  Image<T> Clone() const { return Image(image_.copy_from_me()); }

  // Deep copies the input to this.
  void DeepCopy(Image<T> copy_img);

  // Read image from file.
  void Read(std::string filename) { *this = Image(filename); }

  // Write image to file.
  void Write(const std::string& filename) const {
    CVD::img_save(image_, filename);
  }

  // Get the full Image.
  CVD::Image<T>& GetCVDImage() { return image_; }
  const CVD::Image<T>& GetCVDImage() const { return image_; }

  // Get a pointer to the data.
  T* Data() { return image_.data(); }
  const T* Data() const { return image_.data(); }

  // Extract SubImages from the Image. Note that this does not give ownership of
  // the data to the subimage, the original Image class still owns it.
  // row, col specify the top-left corner of the sub image to extract.
  SubImage<T> GetSubImage(int row, int col, int num_rows, int num_cols);
  const SubImage<T> GetSubImage(int row, int col, int num_rows,
                                int num_cols) const;

  // Gaussian blur.
  void GaussianBlur(double sigma, Image<T>* out);
  void GaussianBlur(double sigma);

  // Sampling techniques.
  void HalfSample(Image<T>* dest) const;
  Image<T> HalfSample() const;
  void TwoThirdsSample(Image<T>* dest) const;
  Image<T> TwoThirdsSample() const;

  // Compute the integral image where pixel (x, y) is equal to the sum of all
  // values in the rectangle from (0, 0) to (x, y).
  template <typename D = T> Image<D> Integrate() const;

  // Resize using a Lanczos 3 filter.
  void Resize(int new_rows, int new_cols);
  void Resize(double scale);
  Image<T> ResizeAndCopy(int new_rows, int new_cols);
  Image<T> ResizeAndCopy(double scale);

 protected:
  friend class SubImage<T>;
  CVD::Image<T> image_;
  using SubImage<T>::sub_image_;

 private:
  // Internal method for resizing images.
  CVD::Image<T> ResizeInternal(int new_rows, int new_cols);

  // Computes gaussian function at x given mu (mean) and sigma;
  inline double Gaussian(double x, double mu, double sigma);

  double lanczos_size_ = 3.0;
  inline double LanczosFilter(double filter_size, double x);
  // Returns the ceiling/floor as an integer.
  inline int CeilInt(double val) { return static_cast<int>(ceil(val)); }
  inline int FloorInt(double val) { return static_cast<int>(floor(val)); }
};

// ----------------- Implementation ----------------- //
// Copy from a CVD image. Only copies the pointer when deep_copy is false.
template <typename T> Image<T>::Image(CVD::Image<T> copy_img, bool deep_copy) {
  image_ = copy_img;
  if (deep_copy) image_.make_unique();
  sub_image_ = image_;
}

// Read from file.
template <typename T> Image<T>::Image(std::string filename) {
  image_ = CVD::img_load(filename);
  image_.make_unique();
  sub_image_ = image_;
}

// Empty Image of size rows, cols.
template <typename T> Image<T>::Image(int rows, int cols) {
  image_.resize(CVD::ImageRef(cols, rows));
  image_.zero();
  sub_image_ = image_;
}

template <typename T> Image<T>::Image(int rows, int cols, T val) {
  image_.resize(CVD::ImageRef(cols, rows));
  image_.fill(val);
  sub_image_ = image_;
}

template <typename T> Image<T> Image<T>::Zeros(int rows, int cols) {
  return Image<T>(rows, cols);
}

// Convert the image to an RGB image.
template <typename T>
template <typename C>
Image<C> Image<T>::ConvertTo() const {
  return Image<C>(CVD::convert_image(image_));
}

// Deep copies the input to this.
template <typename T> void Image<T>::DeepCopy(Image<T> copy_img) {
  image_.copy_from(copy_img.image_);
  sub_image_ = image_;
}

template <typename T>
SubImage<T> Image<T>::GetSubImage(int row, int col, int num_rows,
                                  int num_cols) {
  return SubImage<T>(image_.sub_image(CVD::ImageRef(col, row),
                                      CVD::ImageRef(num_cols, num_rows)));
}

template <typename T>
const SubImage<T> Image<T>::GetSubImage(int row, int col, int num_rows,
                                        int num_cols) const {
  return SubImage<T>(image_.sub_image(CVD::ImageRef(col, row),
                                      CVD::ImageRef(num_cols, num_rows)));
}

template <typename T>
inline double Image<T>::Gaussian(double x, double mu, double sigma) {
  return std::exp(-(x - mu) * (x - mu) / (2.0 * sigma * sigma));
}

template <typename T> void Image<T>::GaussianBlur(double sigma, Image<T>* out) {
  // NOTE: some VERY odd artifacts are occuring with CVD's gaussian
  // convolution with high sigmas...

  // NOTE: This is my own gaussian blur instead. When tested against CVD there
  // is no loss of speed but it avoids any odd ringing artifacts that I was
  // noticing before.
  int kernel_size = static_cast<int>((sigma - 0.8) / 0.3 + 1.0) * 2 + 1;
  int kernel_radius = kernel_size / 2.0;
  std::vector<double> gauss_kernel(kernel_size);
  for (int i = 0; i < gauss_kernel.size(); i++) {
    gauss_kernel[i] = Gaussian(i, (kernel_size - 1.0) / 2.0, sigma);
  }

  // Precompute horizontal filter to leverage row-major memory ordering when
  // applying filter.
  std::vector<double> gauss_terms;
  for (int c = 0; c < this->Cols(); c++) {
    int start_c = std::max(0, c - kernel_radius);
    int end_c = std::min(this->Cols() - 1, c + kernel_radius);
    int last_index = gauss_terms.size();
    double weight = 0;
    for (int i = start_c; i <= end_c; i++) {
      gauss_terms.push_back(gauss_kernel[i - (c - kernel_radius)]);
      weight += gauss_kernel[i - (c - kernel_radius)];
    }
    double inv_weight = 1.0 / weight;
    for (; last_index < gauss_terms.size(); last_index++) {
      gauss_terms[last_index] *= inv_weight;
    }
  }

  // Apply horiz filter.
  CVD::Image<T> horiz_image(image_.size());
  horiz_image.zero();
  for (int r = 0; r < this->Rows(); r++) {
    int gauss_ind = 0;
    for (int c = 0; c < this->Cols(); c++) {
      int start_c = std::max(0, c - kernel_radius);
      int end_c = std::min(this->Cols() - 1, c + kernel_radius);
      for (int cur_col = start_c; cur_col <= end_c; cur_col++) {
        horiz_image[r][c] += image_[r][cur_col] * gauss_terms[gauss_ind++];
      }
    }
  }

  // Apply vertical filter keeping in mind row-major memory for fast processing.
  image_.zero();
  for (int r = 0; r < this->Rows(); r++) {
    int start_r = std::max(0, r - kernel_radius);
    int end_r = std::min(this->Rows() - 1, r + kernel_radius);
    double weight = 0.0;
    for (int cur_row = start_r; cur_row <= end_r; cur_row++) {
      weight += gauss_kernel[cur_row - (r - kernel_radius)];
    }
    double inv_weight = 1.0 / weight;
    for (int cur_row = start_r; cur_row <= end_r; cur_row++) {
      double gauss_term =
          inv_weight * gauss_kernel[cur_row - (r - kernel_radius)];
      for (int c = 0; c < this->Cols(); c++) {
        image_[r][c] += horiz_image[cur_row][c] * gauss_term;
      }
    }
  }
}

template <typename T> void Image<T>::GaussianBlur(double sigma) {
  GaussianBlur(sigma, this);
}

template <typename T> void Image<T>::HalfSample(Image<T>* dest) const {
  CVD::halfSample(image_, dest->image_);
}

template <typename T> Image<T> Image<T>::HalfSample() const {
  return Image<T>(CVD::halfSample(image_));
}

template <typename T> void Image<T>::TwoThirdsSample(Image<T>* dest) const {
  CVD::twoThirdsSample(image_, dest->image_);
}

template <typename T> Image<T> Image<T>::TwoThirdsSample() const {
  return Image<T>(CVD::twoThirdsSample(image_));
}

template <typename T>
template <typename D>
Image<D> Image<T>::Integrate() const {
  CVD::Image<D> integral_image = CVD::integral_image(image_);
  return Image<D>(integral_image);
}

template <typename T> void Image<T>::Resize(int new_rows, int new_cols) {
  CVD::Image<T> new_image = this->ResizeInternal(new_rows, new_cols);
  image_.copy_from(new_image);
  sub_image_ = image_;
}

template <typename T> void Image<T>::Resize(double scale) {
  // Add 0.5 so that it will round to the nearest integer pixel size.
  Resize(scale * this->Rows() + 0.5, scale * this->Cols() + 0.5);
}

template <typename T>
Image<T> Image<T>::ResizeAndCopy(int new_rows, int new_cols) {
  return Image<T>(this->ResizeInternal(new_rows, new_cols), true);
}

template <typename T> Image<T> Image<T>::ResizeAndCopy(double scale) {
  // Add 0.5 so that it will round to the nearest integer pixel size.
  return ResizeAndCopy(scale * this->Rows() + 0.5, scale * this->Cols() + 0.5);
}

// Evaluates the Lanczos filter of the given filter size window for the given
// position.
//
// |filter_size| is the width of the filter (the "window"), outside of which
// the value of the function is 0. Inside of the window, the value is the
// normalized sinc function:
//   lanczos(x) = sinc(x) * sinc(x / filter_size);
// where
//   sinc(x) = sin(pi*x) / (pi*x);
template <typename T>
inline double Image<T>::LanczosFilter(double filter_size, double x) {
  // Outside of the window.
  if (x <= -filter_size || x >= filter_size) return 0.0f;
  // Special case the discontinuity at the origin.
  if (x > -std::numeric_limits<double>::epsilon() &&
      x < std::numeric_limits<double>::epsilon())
    return 1.0f;
  double xpi = x * static_cast<double>(M_PI);
  // sinc(x)*sinc(x/filter_size)
  return (sin(xpi) / xpi) * sin(xpi / filter_size) / (xpi / filter_size);
}

template <typename T>
CVD::Image<T> Image<T>::ResizeInternal(int new_rows, int new_cols) {
  int old_cols = image_.size().x;
  int old_rows = image_.size().y;
  double col_scale =
      static_cast<double>(new_cols) / static_cast<double>(old_cols);
  double row_scale =
      static_cast<double>(new_rows) / static_cast<double>(old_rows);

  double clamped_col_scale = std::min(1.0, col_scale);
  double clamped_row_scale = std::min(1.0, row_scale);
  double inv_col_scale = 1.0 / col_scale;
  double inv_row_scale = 1.0 / row_scale;
  double src_col_support = lanczos_size_ / clamped_col_scale;
  double src_row_support = lanczos_size_ / clamped_row_scale;

  CVD::Image<T> horiz_image(CVD::ImageRef(new_cols, old_rows));
  horiz_image.zero();

  // Precalculate the lanczos kernel so we can take advantage of row-major
  // locality.
  std::vector<double> lanc_kernel_terms;
  for (int c = 0; c < new_cols; c++) {
    // "Overlay" new image onto the old image.
    double src_col = static_cast<double>(c + 0.5) * inv_col_scale;
    // The old col corresponding to the closest new col.
    int src_begin = std::max(0, FloorInt(src_col - src_col_support));
    int src_end = std::min(old_cols - 1, CeilInt(src_col + src_col_support));
    double weight = 0.0;
    int last_index = lanc_kernel_terms.size();
    // Add up terms across the filter.
    for (int cur_col = src_begin; cur_col <= src_end; cur_col++) {
      double src_filter_dist = static_cast<double>(cur_col) + 0.5 - src_col;
      double dest_filter_dist = src_filter_dist * clamped_col_scale;
      double lanc_term = LanczosFilter(lanczos_size_, dest_filter_dist);
      lanc_kernel_terms.push_back(lanc_term);
      weight += lanc_term;
    }
    // Normalize the filter.
    double inv_weight = 1.0 / weight;
    for (; last_index < lanc_kernel_terms.size(); last_index++) {
      lanc_kernel_terms[last_index] *= inv_weight;
    }
  }

  // Using the pre-calculated kernel, apply the filter.
  for (int r = 0; r < old_rows; r++) {
    int i = 0;
    for (int c = 0; c < new_cols; c++) {
      // "Overlay" new image onto the old image.
      double src_col = static_cast<double>(c + 0.5) * inv_col_scale;
      // The old col corresponding to the closest new col.
      int src_begin = std::max(0, FloorInt(src_col - src_col_support));
      int src_end = std::min(old_cols - 1, CeilInt(src_col + src_col_support));
      // Add up terms across the filter.
      for (int cur_col = src_begin; cur_col <= src_end; cur_col++) {
        double lanc_term = lanc_kernel_terms[i++];
        horiz_image[r][c] += image_[r][cur_col] * lanc_term;
      }
    }
  }

  // Now apply a vertical filter to the horiz image.
  CVD::Image<T> new_image(CVD::ImageRef(new_cols, new_rows));
  new_image.zero();
  for (int r = 0; r < new_rows; r++) {
    double src_row = static_cast<double>(r + 0.5) * inv_row_scale;
    int src_begin = std::max(0, FloorInt(src_row - src_row_support));
    int src_end = std::min(old_rows - 1, CeilInt(src_row + src_row_support));
    double weight = 0.0;

    // Calculate the lanczos kernel for this row. Doing it this way allows us to
    // only calculate the kernel values once.
    for (int cur_row = src_begin; cur_row <= src_end; cur_row++) {
      double src_filter_dist = static_cast<double>(cur_row) + 0.5 - src_row;
      double dest_filter_dist = src_filter_dist * clamped_row_scale;
      double lanc_term = LanczosFilter(lanczos_size_, dest_filter_dist);
      weight += lanc_term;
      // Add the kernel contribution. Doing it this way takes advantage of the
      // row-major locality of CVD images.
      for (int c = 0; c < new_cols; c++) {
        new_image[r][c] += horiz_image[cur_row][c] * lanc_term;
      }
    }
    double inv_weight = 1.0 / weight;
    // Fix the weight for all values in this row.
    for (int c = 0; c < new_cols; c++) {
      new_image[r][c] *= inv_weight;
      new_image[r][c] = (new_image[r][c] > 1.0) ? 1.0 : new_image[r][c];
      new_image[r][c] = (new_image[r][c] < 0.0) ? 0.0 : new_image[r][c];
    }
  }
  return new_image;
}

}  // namespace theia

#endif  // THEIA_IMAGE_IMAGE_H_
