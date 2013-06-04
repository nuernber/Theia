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

#ifndef IMAGE_KEYPOINT_DETECTOR_BRISK_HELPER_H_
#define IMAGE_KEYPOINT_DETECTOR_BRISK_HELPER_H_
#include <memory>

namespace theia {
// a layer in the Brisk detector pyramid
class BriskLayer {
 public:
  // constructor arguments
  struct CommonParams {
    static const int HALFSAMPLE = 0;
    static const int TWOTHIRDSAMPLE = 1;
  };
  // construct a base layer
  BriskLayer(const GrayImage& img, float scale, float offset);
  BriskLayer(const GrayImage& img) : BriskLayer(img, 1.0f, 0.0f) {}
  // derive a layer
  BriskLayer(const BriskLayer& layer, int mode);

  // Fast/Agast without non-max suppression
  void getAgastPoints(uint8_t threshold, std::vector<CvPoint>& keypoints);

  // get scores - attention, this is in layer coordinates, not scale=1 coordinates!
  inline uint8_t getAgastScore(int x, int y, uint8_t threshold);
  inline uint8_t getAgastScore_5_8(int x, int y, uint8_t threshold);
  inline uint8_t getAgastScore(float xf, float yf, uint8_t threshold, float scale=1.0f);

  // accessors
  inline const GrayImage& img() const {return img_;}
  inline const GrayImage& scores() const {return scores_;}
  inline float scale() const {return scale_;}
  inline float offset() const {return offset_;}

  // half sampling
  static inline void halfsample(const cv::Mat& srcimg, cv::Mat& dstimg);
  // two third sampling
  static inline void twothirdsample(const cv::Mat& srcimg, cv::Mat& dstimg);

 private:
  // access gray values (smoothed/
  inline uint8_t value(const cv::Mat& mat, float xf, float yf, float scale);
  // the image
  GrayImage img_;
  // its Fast scores
  GrayImage scores_;
  // coordinate transformation
  float scale_;
  float offset_;
  // agast
  std::unique_ptr<agast::OastDetector9_16> oastDetector_;
  std::unique_ptr<agast::AgastDetector5_8> agastDetector_5_8_;
};
}  // namespace theia
#endif  // IMAGE_KEYPOINT_DETECTOR_BRISK_HELPER_H_
