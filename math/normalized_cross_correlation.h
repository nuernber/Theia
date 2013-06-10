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

#ifndef NCC_SSE_H_
#define NCC_SSE_H_
#include <glog/logging.h>
#ifdef THEIA_USE_SSE
#include <xmmintrin.h>
#endif
#include <cmath>

namespace theia {
template<typename T>
T NormalizedCrossCorrelation(const T* a, const T* b, int n) {
  T mean_a = T(0);
  T mean_b = T(0);

  for (int i = 0; i < n; ++i) {
    mean_a += a[i];
    mean_b += b[i];
  }
  mean_a /= n;
  mean_b /= n;

  T sum_aa = T(0);
  T sum_bb = T(0);
  T sum_ab = T(0);

  for (int i = 0; i < n; ++i) {
    T ain = a[i]-mean_a;
    T bin = b[i]-mean_b;
    sum_ab += ain*bin;
    sum_aa += ain*ain;
    sum_bb += bin*bin;
  }
  return sum_ab/sqrt(sum_aa*sum_bb);
}

#ifdef THEIA_USE_SSE
/// Union to switch between SSE and float array.
union sseRegisterHelper {
  __m128 m;
  float f[4];
};

float SSENCC(const float* a, const float* b, int size) {
  const float* a_pt = reinterpret_cast<const float*>(a);
  const float* b_pt = reinterpret_cast<const float*>(b);
  // Helper for non-multiples of 4.
  int lastgroup = size - 3;

  __m128 temp_a, temp_b, a_sum, b_sum;
  float zeros[4] = {0.f, 0.f, 0.f, 0.f};
  a_sum = _mm_load_ps(zeros);
  b_sum = _mm_load_ps(zeros);

  // Sum up the total of each array doing SSE ops in groups of 4 and regular for
  // the rest.
  int i = 0;
  // SSE portion (groups of 4).
  while (i < lastgroup) {
    // Load.
    temp_a = _mm_load_ps(a_pt + i);
    temp_b = _mm_load_ps(b_pt + i);
    // Sum.
    a_sum = _mm_add_ps(a_sum, temp_a);
    b_sum = _mm_add_ps(b_sum, temp_b);
    i += 4;
  }
  sseRegisterHelper res_a, res_b;
  res_a.m = a_sum;
  res_b.m = b_sum;
  // Gather sum.
  float mean_a = (res_a.f[0] + res_a.f[1] +
                  res_a.f[2] + res_a.f[3]);
  float mean_b = (res_b.f[0] + res_b.f[1] +
                  res_b.f[2] + res_b.f[3]);
  while (i < size) {
    mean_a += a_pt[i];
    mean_b += a_pt[i];
    i++;
  }

  // Normalize to mean.
  mean_a /= size;
  mean_b /= size;

  // Calculate mean-adjusted terms.
  float mean_a_f[4] = {mean_a, mean_a, mean_a, mean_a};
  float mean_b_f[4] = {mean_b, mean_b, mean_b, mean_b};
  __m128 sse_mean_a, sse_mean_b, sum_ab, sum_aa, sum_bb, temp_ab;
  sse_mean_a = _mm_load_ps(mean_a_f);
  sse_mean_b = _mm_load_ps(mean_b_f);
  sum_ab = _mm_load_ps(zeros);
  sum_aa = _mm_load_ps(zeros);
  sum_bb = _mm_load_ps(zeros);
  i = 0;
  while (i < lastgroup) {
    // Load.
    temp_a = _mm_load_ps(a_pt + i);
    temp_b = _mm_load_ps(b_pt + i);

    // Subtract mean.
    temp_a = _mm_sub_ps(temp_a, sse_mean_a);
    temp_b = _mm_sub_ps(temp_b, sse_mean_b);

    // Get a*a, b*b, a*b
    temp_ab = _mm_mul_ps(temp_a, temp_b);
    temp_a = _mm_mul_ps(temp_a, temp_a);
    temp_b = _mm_mul_ps(temp_b, temp_b);

    // Add to the running sum.
    sum_ab = _mm_add_ps(sum_ab, temp_ab);
    sum_aa = _mm_add_ps(sum_aa, temp_a);
    sum_bb = _mm_add_ps(sum_bb, temp_b);
    i += 4;
  }
  sseRegisterHelper res_ab, res_aa, res_bb;
  res_ab.m = sum_ab;
  res_aa.m = sum_aa;
  res_bb.m = sum_bb;

  // return sum_ab / sqrt(sum_aa * sum_bb);
  float tot_ab = res_ab.f[0] + res_ab.f[1] + res_ab.f[2] + res_ab.f[3];
  float tot_aa = res_aa.f[0] + res_aa.f[1] + res_aa.f[2] + res_aa.f[3];
  float tot_bb = res_bb.f[0] + res_bb.f[1] + res_bb.f[2] + res_bb.f[3];

  while (i < size) {
    float a_tmp = a[i] - mean_a;
    float b_tmp = b[i] - mean_b;
    tot_ab += a_tmp*b_tmp;
    tot_aa += a_tmp*a_tmp;
    tot_bb += b_tmp*b_tmp;
    i++;
  }

  return tot_ab/sqrt(tot_aa*tot_bb);
}

// Template specialization to run SSE NCC matching.
template <>
float NormalizedCrossCorrelation(const float* a, const float* b, int n) {
  return SSENCC(a, b, n);
}
#endif  // THEIA_USE_SSE
}  // namespace theia
#endif  // NCC_SSE_H_
