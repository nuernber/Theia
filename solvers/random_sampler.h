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

#ifndef SOLVERS_RANDOM_SAMPLER_H_
#define SOLVERS_RANDOM_SAMPLER_H_

#include <algorithm>
#include <chrono>
#include <random>
#include <stdlib.h>
#include <vector>

#include "solvers/sampler.h"

namespace solvers {
// Random sampler used for RANSAC.
template<class Datum>
class RandomSampler : public Sampler<Datum> {
 public:
  // num_samples: the number of samples needed. Typically this corresponds to
  //   the minumum number of samples needed to estimate a model.
  explicit RandomSampler(int num_samples)
      : num_samples_(num_samples),
        generator(std::chrono::system_clock::now().time_since_epoch().count())
  {}

  ~RandomSampler() {}
  // Samples the input variable data and fills the vector subset with the
  // random samples.
  bool Sample(const std::vector<Datum>& data, std::vector<Datum>* subset) {
    std::uniform_int_distribution<int> distribution(0, data.size() - 1);
    subset->resize(num_samples_);
    std::vector<int> random_numbers;
    for (int i = 0; i < num_samples_; i++) {
      int rand_number;
      // Generate a random number that has not already been used.
      while (std::find(random_numbers.begin(),
                       random_numbers.end(),
                       (rand_number = distribution(generator)))
             != random_numbers.end());
      random_numbers.push_back(rand_number);
      subset->at(i) = data[rand_number];
    }
    return true;
  }

 private:
  // Number of samples to obtain.
  int num_samples_;

  // Random number generator engine.
  std::default_random_engine generator;
};
}  // namespace solvers
#endif  // SOLVERS_RANDOM_SAMPLER_H_
