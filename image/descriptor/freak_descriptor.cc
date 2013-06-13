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

#include "image/descriptor/freak_descriptor.h"

#define _USE_MATH_DEFINES
#ifdef THEIA_USE_SSE
#include <emmintrin.h>
#include <tmmintrin.h>
#endif
#include <glog/logging.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "image/image.h"
#include "image/keypoint_detector/keypoint.h"
#ifndef THEIA_NO_PROTOCOL_BUFFERS
#include "image/descriptor/descriptor.pb.h"
#endif
#include "util/util.h"

// It should be noted that this implementation is heavily derived from the
// implementation provided by the authors online at
// http://www.ivpe.com/freak.htm It has been modified significantly to reflect
// the use case for this library (e.g. we assume the default patterns given by
// the authors, rather than train them ourselves).
namespace theia {
namespace {
static const __m128i binMask = _mm_set_epi8(0x80, 0x80, 0x80,
                                            0x80, 0x80, 0x80,
                                            0x80, 0x80, 0x80,
                                            0x80, 0x80, 0x80,
                                            0x80, 0x80, 0x80,
                                            0x80);
static const double kSqrt2 = 1.4142135623731;
static const double kInvSqrt2 = 1.0/kSqrt2;
static const double kLog2 = 0.693147180559945;
static const int kNumOrientation = 256;
static const int kNumPoints = 43;
static const int kSmallestKeypointSize = 7;
// duplicate of Freak member var...
static const int kNumPairs = 512;
static const int freak_def_pairs[kNumPairs] = {
  404, 431, 818, 511, 181, 52, 311, 874, 774, 543, 719, 230, 417, 205, 11,
  560, 149, 265, 39, 306, 165, 857, 250, 8, 61, 15, 55, 717, 44, 412,
  592, 134, 761, 695, 660, 782, 625, 487, 549, 516, 271, 665, 762, 392, 178,
  796, 773, 31, 672, 845, 548, 794, 677, 654, 241, 831, 225, 238, 849, 83,
  691, 484, 826, 707, 122, 517, 583, 731, 328, 339, 571, 475, 394, 472, 580,
  381, 137, 93, 380, 327, 619, 729, 808, 218, 213, 459, 141, 806, 341, 95,
  382, 568, 124, 750, 193, 749, 706, 843, 79, 199, 317, 329, 768, 198, 100,
  466, 613, 78, 562, 783, 689, 136, 838, 94, 142, 164, 679, 219, 419, 366,
  418, 423, 77, 89, 523, 259, 683, 312, 555, 20, 470, 684, 123, 458, 453, 833,
  72, 113, 253, 108, 313, 25, 153, 648, 411, 607, 618, 128, 305, 232, 301, 84,
  56, 264, 371, 46, 407, 360, 38, 99, 176, 710, 114, 578, 66, 372, 653,
  129, 359, 424, 159, 821, 10, 323, 393, 5, 340, 891, 9, 790, 47, 0, 175, 346,
  236, 26, 172, 147, 574, 561, 32, 294, 429, 724, 755, 398, 787, 288, 299,
  769, 565, 767, 722, 757, 224, 465, 723, 498, 467, 235, 127, 802, 446, 233,
  544, 482, 800, 318, 16, 532, 801, 441, 554, 173, 60, 530, 713, 469, 30,
  212, 630, 899, 170, 266, 799, 88, 49, 512, 399, 23, 500, 107, 524, 90,
  194, 143, 135, 192, 206, 345, 148, 71, 119, 101, 563, 870, 158, 254, 214,
  276, 464, 332, 725, 188, 385, 24, 476, 40, 231, 620, 171, 258, 67, 109,
  844, 244, 187, 388, 701, 690, 50, 7, 850, 479, 48, 522, 22, 154, 12, 659,
  736, 655, 577, 737, 830, 811, 174, 21, 237, 335, 353, 234, 53, 270, 62,
  182, 45, 177, 245, 812, 673, 355, 556, 612, 166, 204, 54, 248, 365, 226,
  242, 452, 700, 685, 573, 14, 842, 481, 468, 781, 564, 416, 179, 405, 35,
  819, 608, 624, 367, 98, 643, 448, 2, 460, 676, 440, 240, 130, 146, 184,
  185, 430, 65, 807, 377, 82, 121, 708, 239, 310, 138, 596, 730, 575, 477,
  851, 797, 247, 27, 85, 586, 307, 779, 326, 494, 856, 324, 827, 96, 748,
  13, 397, 125, 688, 702, 92, 293, 716, 277, 140, 112, 4, 80, 855, 839, 1,
  413, 347, 584, 493, 289, 696, 19, 751, 379, 76, 73, 115, 6, 590, 183, 734,
  197, 483, 217, 344, 330, 400, 186, 243, 587, 220, 780, 200, 793, 246, 824,
  41, 735, 579, 81, 703, 322, 760, 720, 139, 480, 490, 91, 814, 813, 163,
  152, 488, 763, 263, 425, 410, 576, 120, 319, 668, 150, 160, 302, 491, 515,
  260, 145, 428, 97, 251, 395, 272, 252, 18, 106, 358, 854, 485, 144, 550,
  131, 133, 378, 68, 102, 104, 58, 361, 275, 209, 697, 582, 338, 742, 589,
  325, 408, 229, 28, 304, 191, 189, 110, 126, 486, 211, 547, 533, 70, 215,
  670, 249, 36, 581, 389, 605, 331, 518, 442, 822};

}  // namespace

// Initializes the sampling patterns and local variables.
bool FreakDescriptorExtractor::Initialize() {
  // If it has already been initialized return.
  if (!pattern_lookup_.empty())
    return true;

  pattern_lookup_.resize(kNumScales_*kNumOrientation*kNumPoints);
  // 2 ^  ((num_octaves_-1) /nbScales)
  double scaleStep =
      std::pow(2.0, static_cast<double>(num_octaves_)/kNumScales_);
  double scalingFactor, alpha, beta, theta = 0;

  // pattern definition, radius normalized to 1.0 (outer point
  // position+sigma=1.0) number of points on each concentric circle (from outer
  // to inner)
  const int n[8] = {6, 6, 6, 6, 6, 6, 6, 1};
  // bigger radius
  const double bigR(2.0/3.0);
  // smaller radius
  const double smallR(2.0/24.0);
  // define spaces between concentric circles (from center to outer:
  // 1,2,3,4,5,6)
  const double unitSpace((bigR-smallR)/21.0);
  // radii of the concentric cirles (from outer to inner)
  const double radius[8] = {bigR, bigR-6*unitSpace, bigR-11*unitSpace,
                            bigR-15*unitSpace, bigR-18*unitSpace,
                            bigR-20*unitSpace, smallR, 0.0};
  // sigma of pattern points (each group of 6 points on a concentric cirle has
  // the same sigma)
  const double sigma[8] = {radius[0]/2.0, radius[1]/2.0, radius[2]/2.0,
                           radius[3]/2.0, radius[4]/2.0, radius[5]/2.0,
                           radius[6]/2.0, radius[6]/2.0
  };
  // fill the lookup table
  for (int scaleIdx = 0; scaleIdx < kNumScales_; ++scaleIdx) {
    // proper initialization
    pattern_sizes_[scaleIdx] = 0;
    // scale of the pattern, scaleStep ^ scaleIdx
    scalingFactor = std::pow(scaleStep, scaleIdx);

    for (int orientationIdx = 0;
         orientationIdx < kNumOrientation;
         ++orientationIdx) {
      // orientation of the pattern
      theta = static_cast<double>(orientationIdx)*2*M_PI/
              static_cast<double>(kNumOrientation);
      int pointIdx = 0;

      PatternPoint* pattern_lookup_Ptr = &pattern_lookup_[0];
      for (size_t i = 0; i < 8; ++i) {
        for (int k = 0 ; k < n[i]; ++k) {
          // orientation offset so that groups of points on each circles are
          // staggered
          beta = M_PI/n[i]*(i%2);
          alpha = k*2.0*M_PI/static_cast<double>(n[i]) + beta + theta;

          // add the point to the look-up table
          PatternPoint& point =
              pattern_lookup_Ptr[scaleIdx*kNumOrientation*kNumPoints +
                                 orientationIdx*kNumPoints+pointIdx];
          point.x = static_cast<float>(radius[i]*cos(alpha)*scalingFactor*
                                       pattern_scale_);
          point.y = static_cast<float>(radius[i]*sin(alpha)*scalingFactor*
                                       pattern_scale_);
          point.sigma =
              static_cast<float>(sigma[i]*scalingFactor* pattern_scale_);

          // adapt the sizeList if necessary
          const int sizeMax = static_cast<int>(
              ceil((radius[i] + sigma[i])* scalingFactor*pattern_scale_)) + 1;
          if (pattern_sizes_[scaleIdx] < sizeMax)
            pattern_sizes_[scaleIdx] = sizeMax;

          ++pointIdx;
        }
      }
    }
  }

  // build the list of orientation pairs
  SetOrientationPair(0, 0, 3);
  SetOrientationPair(1, 1, 4);
  SetOrientationPair(2, 2, 5);
  SetOrientationPair(3, 0, 2);
  SetOrientationPair(4, 1, 3);
  SetOrientationPair(5, 2, 4);
  SetOrientationPair(6, 3, 5);
  SetOrientationPair(7, 4, 0);
  SetOrientationPair(8, 5, 1);

  SetOrientationPair(9, 6, 9);
  SetOrientationPair(10, 7, 10);
  SetOrientationPair(11, 8, 11);
  SetOrientationPair(12, 6, 8);
  SetOrientationPair(13, 7, 9);
  SetOrientationPair(14, 8, 10);
  SetOrientationPair(15, 9, 11);
  SetOrientationPair(16, 10, 6);
  SetOrientationPair(17, 11, 7);

  SetOrientationPair(18, 12, 15);
  SetOrientationPair(19, 13, 16);
  SetOrientationPair(20, 14, 17);
  SetOrientationPair(21, 12, 14);
  SetOrientationPair(22, 13, 15);
  SetOrientationPair(23, 14, 16);
  SetOrientationPair(24, 15, 17);
  SetOrientationPair(25, 16, 12);
  SetOrientationPair(26, 17, 13);

  SetOrientationPair(27, 18, 21);
  SetOrientationPair(28, 19, 22);
  SetOrientationPair(29, 20, 23);
  SetOrientationPair(30, 18, 20);
  SetOrientationPair(31, 19, 21);
  SetOrientationPair(32, 20, 22);
  SetOrientationPair(33, 21, 23);
  SetOrientationPair(34, 22, 18);
  SetOrientationPair(35, 23, 19);

  SetOrientationPair(36, 24, 27);
  SetOrientationPair(37, 25, 28);
  SetOrientationPair(38, 26, 29);
  SetOrientationPair(39, 30, 33);
  SetOrientationPair(40, 31, 34);
  SetOrientationPair(41, 32, 35);
  SetOrientationPair(42, 36, 39);
  SetOrientationPair(43, 37, 40);
  SetOrientationPair(44, 38, 41);

  for (unsigned m = kNumOrientationPairs_; m--;) {
    const float dx = pattern_lookup_[orientation_pairs_[m].i].x -
                     pattern_lookup_[orientation_pairs_[m].j].x;
    const float dy = pattern_lookup_[orientation_pairs_[m].i].y -
                     pattern_lookup_[orientation_pairs_[m].j].y;
    const float norm_sq = (dx*dx + dy*dy);
    orientation_pairs_[m].weight_dx =
        static_cast<int>((dx/(norm_sq))*4096.0 + 0.5);
    orientation_pairs_[m].weight_dy =
        static_cast<int>((dy/(norm_sq))*4096.0 + 0.5);
  }

  // build the list of description pairs
  std::vector<DescriptionPair> allPairs;
  for (unsigned int i = 1; i < (unsigned int)kNumPoints; ++i) {
    // (generate all the pairs)
    for (unsigned int j = 0; (unsigned int)j < i; ++j) {
      DescriptionPair pair = {(uchar)i, (uchar)j};
      allPairs.push_back(pair);
    }
  }
  for (int i = 0; i < kNumPairs_; ++i)
    description_pairs_[i] = allPairs[freak_def_pairs[i]];
  return true;
}

// Computes a descriptor at a single keypoint.
bool FreakDescriptorExtractor::ComputeDescriptor(const GrayImage& image,
                                                 const Keypoint& keypoint,
                                                 FreakDescriptor* descriptor) {
  std::vector<Keypoint*> keypoints;
  // TODO(cmsweeney): Is there a better way to pass the address of the keypoint
  // passed in? This is sort of a lazy hack.
  Keypoint keypoint_copy = keypoint;
  keypoints.push_back(&keypoint_copy);
  std::vector<FreakDescriptor*> descriptors;
  descriptors.push_back(descriptor);
  return ComputeDescriptors(image, keypoints, &descriptors);
}

// Compute multiple descriptors for keypoints from a single image.
bool FreakDescriptorExtractor::ComputeDescriptors(
    const GrayImage& image,
    const std::vector<Keypoint*>& keypoints,
    std::vector<FreakDescriptor*>* descriptors) {
  Image<uchar> uchar_image = image.ConvertTo<uchar>();
  Image<uchar> img_integral = uchar_image.Integrate();

  // used to save pattern scale index corresponding to each keypoints
  std::vector<int> kp_scale_idx(keypoints.size());
  const float size_cst =
      static_cast<float>(kNumScales_/(kLog2*num_octaves_));
  uchar points_value[kNumPoints];
  int theta_idx = 0;
  int direction0;
  int direction1;


  // compute the scale index corresponding to the keypoint size and remove
  // keypoints close to the border.
  descriptors->resize(keypoints.size());
  if (scale_normalized_) {
    for (size_t k = keypoints.size(); k--;) {
      // Is k non-zero? If so, decrement it and continue.
      kp_scale_idx[k] = std::max(static_cast<int>(std::log(
          keypoints[k]->scale()/kSmallestKeypointSize)*size_cst + 0.5), 0);
      if (kp_scale_idx[k] >= kNumScales_)
        kp_scale_idx[k] = kNumScales_ - 1;

      // Check if the description at this specific position and scale fits
      // inside the image.
      if (keypoints[k]->x() <= pattern_sizes_[kp_scale_idx[k]] ||
          keypoints[k]->y() <= pattern_sizes_[kp_scale_idx[k]] ||
          keypoints[k]->x() >= image.Cols() - pattern_sizes_[kp_scale_idx[k]] ||
          keypoints[k]->y() >= image.Rows() - pattern_sizes_[kp_scale_idx[k]]) {
        (*descriptors)[k] = nullptr;
      } else {
        (*descriptors)[k] = new FreakDescriptor;
        (*descriptors)[k]->SetKeypoint(*keypoints[k]);
      }
    }
  } else {
    // equivalent to the formule when the scale is normalized with a constant
    // size of keypoints[k].size=3*SMALLEST_KP_SIZE.
    int scIdx = std::max(static_cast<int>(1.0986122886681*size_cst + 0.5), 0);
    if (scIdx >= kNumScales_) {
      scIdx = kNumScales_ - 1;
    }
    for (size_t k = keypoints.size(); k--;) {
      kp_scale_idx[k] = scIdx;
      if (keypoints[k]->x() <= pattern_sizes_[kp_scale_idx[k]] ||
          keypoints[k]->y() <= pattern_sizes_[kp_scale_idx[k]] ||
          keypoints[k]->x() >= image.Cols() - pattern_sizes_[kp_scale_idx[k]] ||
          keypoints[k]->y() >= image.Rows() - pattern_sizes_[kp_scale_idx[k]]) {
        (*descriptors)[k] = nullptr;
      } else {
        (*descriptors)[k] = new FreakDescriptor;
        (*descriptors)[k]->SetKeypoint(*keypoints[k]);
      }
    }
  }

  // Estimate orientations, extract descriptors, extract the best comparisons
  // only.
  for (size_t k = keypoints.size(); k--;) {
    FreakDescriptor* freak_descriptor = (*descriptors)[k];
    if (freak_descriptor == nullptr) {
      continue;
    }
    // estimate orientation (gradient)
    if (!orientation_normalized_) {
      // assign 0° to all keypoints
      theta_idx = 0;
      freak_descriptor->set_orientation(0.0);
    } else {
      // get the points intensity value in the un-rotated pattern
      for (int i = kNumPoints; i--;) {
        points_value[i] = MeanIntensity(uchar_image, img_integral,
                                        keypoints[k]->x(), keypoints[k]->y(),
                                        kp_scale_idx[k], 0, i);
      }
      direction0 = 0;
      direction1 = 0;
      for (int m = 45; m--;) {
        // iterate through the orientation pairs
        const int delta = points_value[orientation_pairs_[m].i] -
                          points_value[orientation_pairs_[m].j];
        direction0 += delta*(orientation_pairs_[m].weight_dx)/2048;
        direction1 += delta*(orientation_pairs_[m].weight_dy)/2048;
      }
      // estimate orientation
      freak_descriptor->set_orientation(static_cast<float>(
          atan2(static_cast<float>(direction1),
                static_cast<float>(direction0)*(180.0/M_PI))));
      theta_idx = static_cast<int>(
          kNumOrientation*freak_descriptor->orientation()*(1/360.0) + 0.5);
      if (theta_idx < 0)
        theta_idx += kNumOrientation;

      if (theta_idx >= kNumOrientation)
        theta_idx -= kNumOrientation;
    }
    // extract descriptor at the computed orientation
    for (int i = kNumPoints; i--;) {
      points_value[i] = MeanIntensity(uchar_image, img_integral,
                                      keypoints[k]->x(), keypoints[k]->y(),
                                      kp_scale_idx[k], theta_idx, i);
    }

#if THEIA_USE_SSE
    __m128i* ptr = (__m128i*)(freak_descriptor->CharData());
    // NOTE: the comparisons order is modified in each block (but first
    // 128 comparisons remain globally the same-->does not affect the
    // 128,384 bits segmanted matching strategy)
    int cnt = 0;
    for (int n = kNumPairs/128; n-- ; ) {
      __m128i result128 = _mm_setzero_si128();
      for (int m = 128/16; m--; cnt += 16) {
        __m128i operand1 = _mm_set_epi8(
            points_value[description_pairs_[cnt + 0].i],
            points_value[description_pairs_[cnt + 1].i],
            points_value[description_pairs_[cnt + 2].i],
            points_value[description_pairs_[cnt + 3].i],
            points_value[description_pairs_[cnt + 4].i],
            points_value[description_pairs_[cnt + 5].i],
            points_value[description_pairs_[cnt + 6].i],
            points_value[description_pairs_[cnt + 7].i],
            points_value[description_pairs_[cnt + 8].i],
            points_value[description_pairs_[cnt + 9].i],
            points_value[description_pairs_[cnt + 10].i],
            points_value[description_pairs_[cnt + 11].i],
            points_value[description_pairs_[cnt + 12].i],
            points_value[description_pairs_[cnt + 13].i],
            points_value[description_pairs_[cnt + 14].i],
            points_value[description_pairs_[cnt + 15].i]);

        __m128i operand2 = _mm_set_epi8(
            points_value[description_pairs_[cnt + 0].j],
            points_value[description_pairs_[cnt + 1].j],
            points_value[description_pairs_[cnt + 2].j],
            points_value[description_pairs_[cnt + 3].j],
            points_value[description_pairs_[cnt + 4].j],
            points_value[description_pairs_[cnt + 5].j],
            points_value[description_pairs_[cnt + 6].j],
            points_value[description_pairs_[cnt + 7].j],
            points_value[description_pairs_[cnt + 8].j],
            points_value[description_pairs_[cnt + 9].j],
            points_value[description_pairs_[cnt + 10].j],
            points_value[description_pairs_[cnt + 11].j],
            points_value[description_pairs_[cnt + 12].j],
            points_value[description_pairs_[cnt + 13].j],
            points_value[description_pairs_[cnt + 14].j],
            points_value[description_pairs_[cnt + 15].j]);
        // Emulated "not less than" for 8-bit UNSIGNED integers.
        __m128i workReg = _mm_min_epu8(operand1, operand2);
        // Emulated "not less than" for 8-bit UNSIGNED integers.
        workReg = _mm_cmpeq_epi8(workReg, operand2);
        // Merge the last 16 bits with the 128bits std::vector until full.
        workReg = _mm_and_si128(_mm_set1_epi16(short(0x8080 >> m)), workReg);
        result128 = _mm_or_si128(result128, workReg);
      }
      (*ptr) = result128;
      ++ptr;
    }
#else

    // Extracting descriptor preserving the order of SSE version.
    int cnt = 0;
    for (int n = 7; n < kNumPairs; n += 128) {
      for (int m = 8; m--;) {
        int nm = n - m;
        for (int kk = nm + 15*8; kk >= nm; kk -= 8, ++cnt) {
          (*freak_descriptor)[kk] = points_value[description_pairs_[cnt].i] >=
                                    points_value[description_pairs_[cnt].j];
        }
      }
    }
#endif  // THEIA_USE_SSE
  }
  return true;
}

// Simply take average on a square patch, not even gaussian approx.
uchar FreakDescriptorExtractor::MeanIntensity(
    const Image<uchar>& image,
    const Image<uchar>& integral,
    const float kp_x,
    const float kp_y,
    const unsigned int scale,
    const unsigned int rot,
    const unsigned int point) const {
  // get point position in image
  const PatternPoint& freak_point =
      pattern_lookup_[scale*kNumOrientation*kNumPoints +
                      rot*kNumPoints + point];
  const float xf = freak_point.x + kp_x;
  const float yf = freak_point.y + kp_y;
  const int x = static_cast<int>(xf);
  const int y = static_cast<int>(yf);
  const int imagecols = image.Cols();

  // get the sigma:
  const float radius = freak_point.sigma;

  // calculate output:
  if (radius < 0.5) {
    // interpolation multipliers:
    const int r_x = static_cast<int>((xf - x)*1024);
    const int r_y = static_cast<int>((yf - y)*1024);
    const int r_x_1 = 1024 - r_x;
    const int r_y_1 = 1024 - r_y;
    // linear interpolation:
    unsigned int ret_val = r_x_1*r_y_1*image[y][x];
    ret_val = r_x*r_y_1*image[y][x+1];
    ret_val = r_x*r_y*image[y+1][x+1];
    ret_val = r_x_1*r_y*image[y+1][x];
    // return the rounded mean
    ret_val += 2*1024*1024;
    return ret_val/(4*1024*1024);
  }

  // expected case:

  // calculate borders
  const int x_left = static_cast<int>(xf - radius + 0.5);
  const int y_top = static_cast<int>(yf - radius + 0.5);
  // TODO(cmsweeney): decide whether this should be 0.5 or 1.5 because of the
  // integral image.
  const int x_right = static_cast<int>(xf + radius + 0.5);
  const int y_bottom = static_cast<int>(yf + radius + 0.5);

  // bottom right corner
  int ret_val = integral[y_bottom][x_right];
  ret_val -= integral[y_bottom][x_left];
  ret_val += integral[y_top][x_left];
  ret_val -= integral[y_top][x_right];
  ret_val = ret_val/((x_right - x_left)*(y_bottom - y_top));
  return ret_val;
}

// TODO(cmsweeney): write these protos!
#ifndef THEIA_NO_PROTOCOL_BUFFERS
bool FreakDescriptorExtractor::ProtoToDescriptor(
    const DescriptorsProto& proto,
    std::vector<FreakDescriptor*>* descriptors) const {
  descriptors->reserve(proto.feature_descriptor_size());
  for (const DescriptorProto& proto_descriptor: proto.feature_descriptor()) {
    FreakDescriptor* descriptor = new FreakDescriptor;
    CHECK_EQ(proto_descriptor.descriptor_type(), DescriptorProto::FREAK)
        << "Descriptor in proto is not a patch descriptor.";
    CHECK_EQ(proto_descriptor.float_descriptor_size(), descriptor->Dimensions())
        << "Dimension mismatch in the proto and descriptors.";
    descriptor->set_x(proto_descriptor.x());
    descriptor->set_y(proto_descriptor.y());
    descriptor->set_orientation(proto_descriptor.orientation());
    descriptor->set_scale(proto_descriptor.scale());
    if (proto_descriptor.has_strength())
      descriptor->set_strength(proto_descriptor.strength());

    // Get float array.
    for (int i = 0; i < descriptor->Dimensions(); i++)
      (*descriptor)[i] = proto_descriptor.float_descriptor(i);
    descriptors->push_back(descriptor);
  }
  return true;
}

bool FreakDescriptorExtractor::DescriptorToProto(
    const std::vector<FreakDescriptor*>& descriptors,
    DescriptorsProto* proto) const {
  for (const FreakDescriptor* descriptor : descriptors) {
    DescriptorProto* descriptor_proto = proto->add_feature_descriptor();
    // Add the float array to the proto.
    for (int i = 0; i < descriptor->Dimensions(); i++)
      descriptor_proto->add_float_descriptor((*descriptor)[i]);
    // Set the proto type to patch.
    descriptor_proto->set_descriptor_type(DescriptorProto::FREAK);
    descriptor_proto->set_x(descriptor->x());
    descriptor_proto->set_y(descriptor->y());
    descriptor_proto->set_orientation(descriptor->orientation());
    descriptor_proto->set_scale(descriptor->scale());
    if (descriptor->has_strength())
      descriptor_proto->set_strength(descriptor->strength());
  }
  return true;
}
#endif
}  // namespace theia
