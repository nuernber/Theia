// Copyright (C) 2013  Chris Sweeney <cmsweeney@cs.ucsb.edu>
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
//     * Neither the name of the University of California, Santa Barbara nor the
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

#ifndef SOLVERS_PROSAC_SAMPLER_H_
#define SOLVERS_PROSAC_SAMPLER_H_

#include <algorithm>
#include <chrono>
#include <glog/logging.h>
#include <random>
#include <stdlib.h>
#include <vector>

#include "solvers/sampler.h"

namespace solvers {
// Prosac sampler used for PROSAC implemented according to "Matching with PROSAC
// - Progressive Sampling Consensus" by Chum and Matas.
template<class Datum>
class ProsacSampler : public Sampler<Datum> {
 public:
  // num_samples: the number of samples needed. Typically this corresponds to
  //   the minumum number of samples needed to estimate a model.
  explicit ProsacSampler(int num_samples,
                         int ransac_convergence_iterations = 200000)
      : num_samples_(num_samples),
        ransac_convergence_iterations_(ransac_convergence_iterations),
        kth_sample_number_(1),
        generator(std::chrono::system_clock::now().time_since_epoch().count())
  {}
  
  ~ProsacSampler() {}

  // Set the sample such that you are sampling the kth prosac sample (Eq. 6).
  void SetSampleNumber(int k) {
    kth_sample_number_ = k;
  }

  // Samples the input variable data and fills the vector subset with the prosac
  // samples.
  // NOTE: This assumes that data is in sorted order by quality where data[i] is
  // of higher quality than data[j] for all i < j.
  bool Sample(const std::vector<Datum>& data, std::vector<Datum>* subset) {
    // Set t_n according to the PROSAC paper's recommendation.
    double t_n = ransac_convergence_iterations_;
    int n = num_samples_;
    // From Equations leading up to Eq 3 in Chum et al.
    for (int i = 0; i < num_samples_; i++) {
      t_n *= static_cast<double>(n - i)/(data.size() - i);
    }

    double t_n_prime = 1.0;
    // Choose min n such that T_n_prime >= t (Eq. 5).
    for (int t = 1; t <= kth_sample_number_; t++) {
      if (t > t_n_prime && n < data.size()) {
        double t_n_plus1 = (t_n*(n + 1.0))/(n + 1.0 - num_samples_);
        t_n_prime += ceil(t_n_plus1 - t_n);
        t_n = t_n_plus1;
        n++;
      }
    }


    subset->resize(num_samples_);
    if (t_n_prime < kth_sample_number_) {
      // Randomly sample m data points from the top n data points.
      std::uniform_int_distribution<int> distribution(0, n - 1);
      std::vector<int> random_numbers;
      for (int i = 0; i < num_samples_; i++) {
        // Generate a random number that has not already been used.
        int rand_number;
        while (std::find(random_numbers.begin(),
                         random_numbers.end(),
                         (rand_number = distribution(generator)))
               != random_numbers.end());
        random_numbers.push_back(rand_number);

        // Push the *unique* random index back.
        subset->at(i) = data[rand_number];
      }
    } else {
      std::uniform_int_distribution<int> distribution(0, n - 2);
      std::vector<int> random_numbers;
      // Randomly sample m-1 data points from the top n-1 data points.
      for (int i = 0; i < num_samples_ - 1; i++) {
        // Generate a random number that has not already been used.
        int rand_number;
        while (std::find(random_numbers.begin(),
                         random_numbers.end(),
                         (rand_number = distribution(generator)))
               != random_numbers.end());
        random_numbers.push_back(rand_number);

        // Push the *unique* random index back.
        subset->at(i) = data[rand_number];
      }
      // Make the last point from the nth position.
      subset->at(num_samples_ - 1) = data[n];
    }
    CHECK_EQ(subset->size(), num_samples_) << "Prosac subset is incorrect "
                                           << "size!";
    kth_sample_number_++;
    return true;
  }

 private:
  // The kth sample of
  int kth_sample_number_;

  // Number of samples to return.
  int num_samples_;

  // Number of iterations of PROSAC before it just acts like ransac.
  int ransac_convergence_iterations_;

  // Random number generator seed
  std::default_random_engine generator;
};
}  // namespace solvers
#endif  // SOLVERS_PROSAC_SAMPLER_H_
