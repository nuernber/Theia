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

#ifndef VISION_MATCHING_BRUTE_FORCE_MATCHER_H_
#define VISION_MATCHING_BRUTE_FORCE_MATCHER_H_

#include <vector>

#include "vision/matching/matcher.h"
#include "util/util.h"

namespace theia {
// Template Descriptor can be any type of descriptor, and Metric can be any of
// the distance metrics defined in distance.h
template <class TemplateDescriptor, class Metric>
class BruteForceMatcher : public Matcher<TemplateDescriptor, Metric> {
 public:
  typedef typename Metric::ResultType TDistanceType;

  BruteForceMatcher() {}
  ~BruteForceMatcher() {}

  // Build an index or other data structures needed to perform the search.
  bool Build(const std::vector<TemplateDescriptor*>& descriptors) {
    descriptors_ = descriptors;
  }
  
  // Search for the sole nearest neighbor for a single query.
  bool NearestNeighbor(const TemplateDescriptor& query,
                       int* neighbor_index,
                       TDistanceType* distance,
                       TDistanceType threshold) {
    Metric metric;
    *neighbor_index = 0;
    *distance = metric(query.Data(),
                       descriptors_[0]->Data(),
                       query.Dimensions());
    
    // Set a threshold if there isn't one already set.
    if (threshold <= 0)
      threshold = *distance;
    for (int i = 1; i < descriptors_.size(); i++) {
      TDistanceType new_dist = metric(query.Data(),
                                      descriptors_[i]->Data(),
                                      query.Dimensions());
      if (new_dist < *distance && new_dist < threshold) {
        *neighbor_index = i;
        *distance = new_dist;
      }
    }
    return true;
  }

  // This allows us to use the overloaded function in the base class (passing in
  // a vector of queries instead of just one).
  using Matcher<TemplateDescriptor, Metric>::NearestNeighbor;

  // Search for the k nearest neighbors of a single queries.
  bool KNearestNeighbors(const TemplateDescriptor& query,
                         int k_nn,
                         std::vector<int>* knn_index,
                         std::vector<TDistanceType>* knn_distance) {
  }

  // This allows us to use the overloaded function in the base class (passing in
  // a vector of queries instead of just one).
  using Matcher<TemplateDescriptor, Metric>::KNearestNeighbors;
  
 private:
  std::vector<TemplateDescriptor*> descriptors_;
  DISALLOW_COPY_AND_ASSIGN(BruteForceMatcher);
};
}  // namespace theia
#endif  // VISION_MATCHING_BRUTE_FORCE_MATCHER_H_
