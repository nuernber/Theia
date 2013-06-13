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

#ifndef VISION_MATCHING_DISTANCE_HAMMING_H_
#define VISION_MATCHING_DISTANCE_HAMMING_H_

#ifdef THEIA_USE_SSE
#include <emmintrin.h>
#include <tmmintrin.h>
#endif

#include <bitset>
#include "util/util.h"

namespace theia {
#ifdef THEIA_USE_SSE
#ifdef __GNUC__
static const char __attribute__((aligned(16))) MASK[16] = {0xf, 0xf, 0xf, 0xf,
                                                           0xf, 0xf, 0xf, 0xf,
                                                           0xf, 0xf, 0xf, 0xf,
                                                           0xf, 0xf, 0xf, 0xf};
static const uint8_t __attribute__((aligned(16))) LUT_POP[16] = { 0, 1, 1, 2,
                                                                  1, 2, 2, 3,
                                                                  1, 2, 2, 3,
                                                                  2, 3, 3, 4};
static const __m128i SHIFT = _mm_set_epi32(0, 0, 0, 4);
#endif

#ifdef _MSC_VER
__declspec(align(16)) static const char MASK[16] = {0xf, 0xf, 0xf, 0xf,
                                                    0xf, 0xf, 0xf, 0xf,
                                                    0xf, 0xf, 0xf, 0xf,
                                                    0xf, 0xf, 0xf, 0xf};
__declspec(align(16)) static const uint8_t LUT_POP[16] = { 0, 1, 1, 2,
                                                           1, 2, 2, 3,
                                                           1, 2, 2, 3,
                                                           2, 3, 3, 4};
static const __m128i SHIFT = _mm_set_epi32(0, 0, 0, 4);
#endif

/** Hamming distance using SSE instruction with intermediat threshold @param Tr
 * intermediate threshold on the first 128 bits allows ~25% speed gain while not
 * affecting the final result too restrictive value will reject true positive
 * too high value will increase the computation time (no early rejection, but an
 * extra condition tested anyway) for optimal efficiency make sure the 512
 * description pairs have been selected accordingly to the detector values out
 * of range [0,128] have no effect @param nbBlocks number of blocks of 128 bits
 */

// 512 hamming distance segmented in 128+384 bits SSSE3 adapted from
// http://wm.ite.pl/articles/sse-popcount.html and BRISK: Binary Robust
// Invariant Scalable Keypoints :
// http://www.asl.ethz.ch/people/lestefan/personal/BRISK
// http://en.wikipedia.org/wiki/Hamming_weight
template<int Tr = 0, int nbBlocks = 4>
static int XORedPopcnt_128_384(const __m128i* string1, const __m128i* string2) {
  register __m128i xmm0;
  register __m128i xmm1;
  // vector of popcount for lower nibbles
  register __m128i xmm2;
  // vector of popcount for higher nibbles
  register __m128i xmm3;
  // local accumulator (for inner loop)
  register __m128i xmm4;
  // global accumulator
  register __m128i xmm5;
  // MASK
  register __m128i xmm6;
  // LUT_POP
  register __m128i xmm7;
  register __m128i xmm8;
  // Returns the value loaded in a variable representing a register
  xmm7 = _mm_load_si128(reinterpret_cast<__m128i*>(LUT_POP));
  xmm6 = _mm_load_si128(reinterpret_cast<__m128i*>(MASK));
  // Sets the 128-bit value to zero
  xmm5 = _mm_setzero_si128();
  xmm4 = xmm5;

  // Computes the bitwise XOR of the 128-bit value in a and the 128-bit value in
  // b.
  xmm0 = _mm_xor_si128((__m128i)*string1++, (__m128i)*string2++);
  xmm1 = xmm0;
  // Shifts the 8 signed or unsigned 16-bit integers in a right by count bits
  // while shifting in zeros.
  xmm1 = _mm_srl_epi16(xmm1, SHIFT);
  // Computes the bitwise AND of the 128-bit value in a and the 128-bit value in
  // b.
  xmm0 = _mm_and_si128(xmm0, xmm6);
  xmm1 = _mm_and_si128(xmm1, xmm6);
  xmm2 = xmm7;
  xmm3 = xmm7;
  // lower nibbles.Emits the Supplemental Streaming SIMD Extensions 3 (SSSE3)
  // instruction pshufb.
  xmm2 = _mm_shuffle_epi8(xmm2, xmm0);
  // This instruction shuffles 16-byte parameters from a 128-bit parameter.
  // higher nibbles
  xmm3 = _mm_shuffle_epi8(xmm3, xmm1);
  // Adds the 16 signed or unsigned 8-bit integers in a to the 16 signed or
  // unsigned 8-bit integers in b.
  xmm4 = _mm_add_epi8(xmm4, xmm2);
  xmm4 = _mm_add_epi8(xmm4, xmm3);

  // Early out if applicable.
  if (Tr < 128 && Tr > 0) {
    xmm8 = _mm_sad_epu8(xmm4, xmm5);
    xmm0 = (__m128i) _mm_movehl_ps((__m128) xmm0, (__m128) xmm8);
    xmm0 = _mm_add_epi32(xmm0, xmm8);
    if ( _mm_cvtsi128_si32(xmm0) > Tr ) {
      return 999999999;
    }
  }

  // Computes the bitwise XOR of the 128-bit value in a and the 128-bit value in
  // b.
  xmm0 = _mm_xor_si128((__m128i)*string1++, (__m128i)*string2++);
  xmm1 = xmm0;
  // Shifts the 8 signed or unsigned 16-bit integers in a right by count bits
  // while shifting in zeros.
  xmm1 = _mm_srl_epi16(xmm1, SHIFT);
  // Computes the bitwise AND of the 128-bit value in a and the 128-bit value in
  // b.
  xmm0 = _mm_and_si128(xmm0, xmm6);
  xmm1 = _mm_and_si128(xmm1, xmm6);
  xmm2 = xmm7;
  xmm3 = xmm7;
  xmm2 = _mm_shuffle_epi8(xmm2, xmm0);
  // higher nibbles
  xmm3 = _mm_shuffle_epi8(xmm3, xmm1);
  // Adds the 16 signed or unsigned 8-bit integers in a to the 16 signed or
  // unsigned 8-bit integers in b.
  xmm4 = _mm_add_epi8(xmm4, xmm2);
  xmm4 = _mm_add_epi8(xmm4, xmm3);

  // Computes the bitwise XOR of the 128-bit value in a and the 128-bit value in
  // b.
  xmm0 = _mm_xor_si128((__m128i)*string1++, (__m128i)*string2++);
  xmm1 = xmm0;
  // Shifts the 8 signed or unsigned 16-bit integers in a right by count bits
  // while shifting in zeros.
  xmm1 = _mm_srl_epi16(xmm1, SHIFT);
  // Computes the bitwise AND of the 128-bit value in a and the 128-bit value in
  // b.
  xmm0 = _mm_and_si128(xmm0, xmm6);
  xmm1 = _mm_and_si128(xmm1, xmm6);
  xmm2 = xmm7;
  xmm3 = xmm7;
  xmm2 = _mm_shuffle_epi8(xmm2, xmm0);
  // higher nibbles
  xmm3 = _mm_shuffle_epi8(xmm3, xmm1);
  // Adds the 16 signed or unsigned 8-bit integers in a to the 16 signed or
  // unsigned 8-bit integers in b.
  xmm4 = _mm_add_epi8(xmm4, xmm2);
  xmm4 = _mm_add_epi8(xmm4, xmm3);

  // Computes the bitwise XOR of the 128-bit value in a and the 128-bit value in
  // b.
  xmm0 = _mm_xor_si128((__m128i)*string1++, (__m128i)*string2++);
  xmm1 = xmm0;
  // Shifts the 8 signed or unsigned 16-bit integers in a right by count bits
  // while shifting in zeros.
  xmm1 = _mm_srl_epi16(xmm1, SHIFT);
  // Computes the bitwise AND of the 128-bit value in a and the 128-bit value in
  // b.
  xmm0 = _mm_and_si128(xmm0, xmm6);
  xmm1 = _mm_and_si128(xmm1, xmm6);
  xmm2 = xmm7;
  xmm3 = xmm7;
  // lower nibbles
  xmm2 = _mm_shuffle_epi8(xmm2, xmm0);
  // higher nibbles
  xmm3 = _mm_shuffle_epi8(xmm3, xmm1);
  // Adds the 16 signed or unsigned 8-bit integers in a to the 16 signed or
  // unsigned 8-bit integers in b.
  xmm4 = _mm_add_epi8(xmm4, xmm2);
  xmm4 = _mm_add_epi8(xmm4, xmm3);

  if (nbBlocks > 4) {
    for (int i = nbBlocks-4; i--;) {
      // Computes the bitwise XOR of the 128-bit value in a and the 128-bit
      // value in b.
      xmm0 = _mm_xor_si128((__m128i)*string1++, (__m128i)*string2++);
      xmm1 = xmm0;
      // Shifts the 8 signed or unsigned 16-bit integers in a right by count
      // bits while shifting in zeros.
      xmm1 = _mm_srl_epi16(xmm1, SHIFT);
      // Computes the bitwise AND of the 128-bit value in a and the 128-bit
      // value in b.
      xmm0 = _mm_and_si128(xmm0, xmm6);
      xmm1 = _mm_and_si128(xmm1, xmm6);
      xmm2 = xmm7;
      xmm3 = xmm7;
      // lower nibbles
      xmm2 = _mm_shuffle_epi8(xmm2, xmm0);
      // higher nibbles
      xmm3 = _mm_shuffle_epi8(xmm3, xmm1);
      // Adds the 16 signed or unsigned 8-bit integers in a to the 16 signed or
      // unsigned 8-bit integers in b.
      xmm4 = _mm_add_epi8(xmm4, xmm2);
      xmm4 = _mm_add_epi8(xmm4, xmm3);
    }
  }

  // Computes the absoLUT_POPe difference of the 16 unsigned 8-bit integers from
  // a and the 16 unsigned 8-bit integers from b.
  xmm4 = _mm_sad_epu8(xmm4, xmm5);
  // Sums the upper 8 differences and lower 8 differences and packs the
  // resulting 2 unsigned 16-bit integers into the upper and lower 64-bit
  // elements.

  xmm0 = (__m128i) _mm_movehl_ps((__m128) xmm0, (__m128) xmm4);
  xmm0 = _mm_add_epi32(xmm0, xmm4);
  // Moves the least significant 32 bits of a to a 32-bit integer.
  return _mm_cvtsi128_si32(xmm0);
}
#endif  // THEIA_USE_SSE

struct Hamming {
  typedef bool ElementType;
  typedef int ResultType;

  template <typename Iterator1, typename Iterator2>
  ResultType operator()(const Iterator1 a, const Iterator2 b,
                        const size_t size) const {
    // a^b = XOR op, then perform a popcnt on it. This should be optimized
    // rather heavily, and is architecture independent since it is part of the
    // c++ standard. NOTE: cmsweeney tested bitset count vs SSE from BRISK and
    // observed bitset count was roughly ~ 50% faster.
    int dist1 = (*a ^ *b).count();
    return dist1;
  }
};

// max_early_dist is the maximum value of the distance of the first 128 bits in
// order to continue testing the distance.
//
// NOTE: we have found from experiments that this the SSE version is slower than
// using a modified bitset version.
template<int max_early_dist = 0>
struct HammingFreak {
  typedef bool ElementType;
  typedef int ResultType;

  template <typename Iterator1, typename Iterator2>
  ResultType operator()(Iterator1 a, Iterator2 b, size_t size) const {
    // #ifndef THEIA_USE_SSE

    // Test the distance of the first 128 bytes. This cast
    // seems more dangerous than it is. a and b are just reinterpretted casts of
    // uchar arrays to begin with, so we do not risk messing up with internals
    // by casting it to a smaller bitset for our early out.
    const std::bitset<128>* a_first =
        reinterpret_cast<const std::bitset<128>*>(a);
    const std::bitset<128>* b_first =
        reinterpret_cast<const std::bitset<128>*>(b);
    if (max_early_dist > 0 && (*a_first ^ *b_first).count() > max_early_dist) {
      return size;
    } else {
      // Otherwise, return the normal distance.
      return (*a ^ *b).count();
    }
    /*
      #else
      const uchar* a_char = reinterpret_cast<const uchar*>(a);
      const uchar* b_char = reinterpret_cast<const uchar*>(b);
      return XORedPopcnt_128_384<max_early_dist>((const __m128i*)a_char,
      (const __m128i*)b_char);
      #endif  // THEIA_USE_SSE
    */
  }
};

}  // namespace theia
#endif  // VISION_MATCHING_DISTANCE_HAMMING_H_
