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

#ifndef VISION_MATCHING_DISTANCE_H_
#define VISION_MATCHING_DISTANCE_H_

#include <glog/logging.h>
#ifdef THEIA_USE_SSE
#include <xmmintrin.h>
#endif

namespace theia {
// This file includes all of the distance metrics that are used:
// L2 distance for euclidean features.
// Vectorized L2 distance for SSE optimizations.
// Hamming distance for binary vectors.
// Hamming distance for FREAK (checks the first 128 bits, then checks the rest).
// NOTE: Hamming distance functions are included at the bottom of this file, but
// are in a separate file for cleanliness.
template<typename T>
struct Accumulator { typedef T Type; };

template<>
struct Accumulator<unsigned char>  { typedef float Type; };

template<>
struct Accumulator<unsigned short> { typedef float Type; };

template<>
struct Accumulator<unsigned int> { typedef float Type; };

template<>
struct Accumulator<char>   { typedef float Type; };

template<>
struct Accumulator<short>  { typedef float Type; };

template<>
struct Accumulator<int> { typedef float Type; };

template<>
struct Accumulator<bool> { typedef int Type; };

// Squared Euclidean distance functor (copied from FLANN).
template<class T>
struct L2 {
  typedef T ElementType;
  typedef typename Accumulator<T>::Type ResultType;

  template <typename Iterator1, typename Iterator2>
  ResultType operator()(Iterator1 a, Iterator2 b, size_t size) const {
    ResultType result = ResultType();
    ResultType diff;
    for (size_t i = 0; i < size; ++i) {
      diff = *a++ - *b++;
      result += diff*diff;
    }
    return result;
  }
};

// Squared Euclidean distance functor (vectorized version).
template<class T>
struct L2Vectorized {
  typedef T ElementType;
  typedef typename Accumulator<T>::Type ResultType;

  template <typename Iterator1, typename Iterator2>
  ResultType operator()(Iterator1 a, Iterator2 b, size_t size) const {
    ResultType result = ResultType();
    ResultType diff0, diff1, diff2, diff3;
    Iterator1 last = a + size;
    Iterator1 lastgroup = last - 3;

    // Process 4 items with each loop for efficiency.
    while (a < lastgroup) {
      diff0 = a[0] - b[0];
      diff1 = a[1] - b[1];
      diff2 = a[2] - b[2];
      diff3 = a[3] - b[3];
      result += diff0*diff0 + diff1*diff1 + diff2*diff2 + diff3*diff3;
      a += 4;
      b += 4;
    }
    // Process last 0-3 dimensions.  Not needed for standard vector lengths.
    while (a < last) {
      diff0 = *a++ - *b++;
      result += diff0*diff0;
    }
    return result;
  }
};

#ifdef THEIA_USE_SSE
/// Union to switch between SSE and float array.
union sseRegisterHelper {
  __m128 m;
  float f[4];
};

// Euclidian distance (SSE method) (squared result)
float L2SSE(float* b1, float* b2, int size) {
  CHECK_EQ(size%4, 0) << "Size must be a multiple of 4 for SSE optimization!";
  float* b1Pt = reinterpret_cast<float*>(b1);
  float* b2Pt = reinterpret_cast<float*>(b2);
  __m128 srcA, srcB, temp, cumSum;
  float zeros[4] = {0.f, 0.f, 0.f, 0.f};
  cumSum = _mm_load_ps(zeros);
  for (int i = 0; i < size; i += 4) {
    srcA = _mm_load_ps(b1Pt + i);
    srcB = _mm_load_ps(b2Pt + i);
    // Substract
    temp = _mm_sub_ps(srcA, srcB);
    // Multiply
    temp =  _mm_mul_ps(temp, temp);
    // sum
    cumSum = _mm_add_ps(cumSum, temp);
  }
  sseRegisterHelper res;
  res.m = cumSum;
  return (res.f[0] + res.f[1] + res.f[2] + res.f[3]);
}
// Template specialization to run SSE L2 squared distance on float vector.
template<>
struct L2Vectorized<float> {
  typedef float ElementType;
  typedef Accumulator<float>::Type ResultType;

  template <typename Iterator1, typename Iterator2>
  ResultType operator()(Iterator1 a, Iterator2 b, size_t size) const {
    return L2SSE(a, b, size);
  }
};
#endif  // THEIA_USE_SSE
}  // namespace theia

// Include the hamming distance functions in a separate file for cleanliness.
#include "vision/matching/distance_hamming.h"

#endif  // VISION_MATCHING_DISTANCE_H_
