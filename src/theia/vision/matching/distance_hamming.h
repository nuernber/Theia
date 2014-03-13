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

#ifndef THEIA_VISION_MATCHING_DISTANCE_HAMMING_H_
#define THEIA_VISION_MATCHING_DISTANCE_HAMMING_H_

#ifdef THEIA_USE_SSE
#include <emmintrin.h>
#include <tmmintrin.h>
#endif
#include <functional>
#include <numeric>

#include "theia/util/util.h"

namespace theia {

#ifdef THEIA_USE_SSE

#ifdef __GNUC__
static const char __attribute__((aligned(16)))
    MASK_4bit[16] = { 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf,
                      0xf, 0xf, 0xf, 0xf, 0xf };
static const uint8_t __attribute__((aligned(16)))
    POPCOUNT_4bit[16] = { 0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4 };
static const __m128i shiftval = _mm_set_epi32(0, 0, 0, 4);
#endif
#ifdef _MSC_VER
__declspec(align(16)) static const char MASK_4bit[16] = {
  0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf
};
__declspec(align(16)) static const uint8_t POPCOUNT_4bit[16] = {
  0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4
};
static const __m128i shiftval = _mm_set_epi32(0, 0, 0, 4);
#endif

// - SSSE3 - better alorithm, minimized psadbw usage - adapted from
// http://wm.ite.pl/articles/sse-popcount.html
inline uint32_t HammingSSE(const uchar* a, const uchar*b,
                           const int size) {
  uint32_t result = 0;

  const __m128i* signature1 = reinterpret_cast<const __m128i*>(a);
  const __m128i* signature2 = reinterpret_cast<const __m128i*>(b);
  const int number_of_128_bit_words = size / 128;

  __m128i xmm0;
  __m128i xmm1;
  __m128i xmm2;
  __m128i xmm3;
  __m128i xmm4;
  __m128i xmm5;
  __m128i xmm6;
  __m128i xmm7;

  xmm7 = _mm_load_si128(reinterpret_cast<const __m128i*>(POPCOUNT_4bit));
  xmm6 = _mm_load_si128(reinterpret_cast<const __m128i*>(MASK_4bit));

  // accumulator
  xmm5 = _mm_setzero_si128();

  const size_t end =
      reinterpret_cast<const size_t>(signature1 + number_of_128_bit_words);

  xmm4 = xmm5;

  // Loop over 128 bit words.
  do {
    xmm0 = _mm_xor_si128((__m128i) * signature1++, (__m128i) * signature2++);

    xmm1 = xmm0;

    xmm1 = _mm_srl_epi16(xmm1, shiftval);
    xmm0 = _mm_and_si128(xmm0, xmm6);
    xmm1 = _mm_and_si128(xmm1, xmm6);
    xmm2 = xmm7;
    xmm3 = xmm7;
    xmm2 = _mm_shuffle_epi8(xmm2, xmm0);
    xmm3 = _mm_shuffle_epi8(xmm3, xmm1);
    xmm4 = _mm_add_epi8(xmm4, xmm2);
    xmm4 = _mm_add_epi8(xmm4, xmm3);
  } while ((size_t) signature1 < end);

  xmm4 = _mm_sad_epu8(xmm4, xmm5);
  xmm5 = _mm_add_epi32(xmm5, xmm4);

  // Add together 32-bits counters stored in global accumulator
  xmm0 = _mm_cvtps_epi32(
      _mm_movehl_ps(_mm_cvtepi32_ps(xmm0), _mm_cvtepi32_ps(xmm5)));

  xmm0 = _mm_add_epi32(xmm0, xmm5);
  result = _mm_cvtsi128_si32(xmm0);
  return result;
}

struct Hamming {
  typedef int ResultType;

  ResultType operator()(const Descriptor& a, const Descriptor& b) const {
    return HammingSSE(a.CharData(), b.CharData(), a.Dimensions());
  }
};

#else

struct Hamming {
  typedef int ResultType;

  ResultType operator()(const Descriptor& a, const Descriptor& b) const {
    static constexpr uchar pop_count_table[] = {
      0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4, 2,
      3, 3, 4, 3, 4, 4, 5, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3,
      3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3,
      4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 2, 3, 3, 4,
      3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5,
      6, 6, 7, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4,
      4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5,
      6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 2, 3, 3, 4, 3, 4, 4, 5,
      3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 3,
      4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 4, 5, 5, 6, 5, 6, 6, 7, 5, 6,
      6, 7, 6, 7, 7, 8
    };

    int result = 0;
    const uchar* char_a = a.CharData();
    const uchar* char_b = b.CharData();
    for (size_t i = 0; i < a.Dimensions() / (8 * sizeof(uchar)); i++) {
      result += pop_count_table[char_a[i] ^ char_b[i]];
    }
    return result;
  }
};
#endif

}  // namespace theia

#endif  // THEIA_VISION_MATCHING_DISTANCE_HAMMING_H_
