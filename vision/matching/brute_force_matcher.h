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

#include <queue>
#include <vector>

#include "vision/matching/matcher.h"
#include "util/util.h"

namespace theia {
// Template Descriptor can be any type of descriptor, and Metric can be any of
// the distance metrics defined in distance.h
template <class TDescriptor, class Metric>
class BruteForceMatcher : public Matcher<TDescriptor, Metric> {
 public:
  typedef typename Metric::ResultType DistanceType;
  typedef TDescriptor DescriptorType;

  BruteForceMatcher() {}
  ~BruteForceMatcher() {}

  // Build an index or other data structures needed to perform the search.
  bool Build(const std::vector<TDescriptor*>& descriptors) {
    descriptors_ = descriptors;
  }

  // Search for the sole nearest neighbor for a single query.
  bool NearestNeighbor(const TDescriptor& query,
                       int* neighbor_index,
                       DistanceType* distance,
                       DistanceType threshold) {
    Metric metric;
    *neighbor_index = 0;
    *distance = metric(query.Data(),
                       descriptors_[0]->Data(),
                       query.Dimensions());
    bool found_nn = false;
    // Set a threshold if there isn't one already set.
    if (threshold <= 0)
      threshold = *distance;

    if (*distance <= threshold)
      found_nn = true;

    for (int i = 1; i < descriptors_.size(); i++) {
      DistanceType new_dist = metric(query.Data(),
                                     descriptors_[i]->Data(),
                                     query.Dimensions());
      if (new_dist < *distance && new_dist < threshold) {
        *neighbor_index = i;
        *distance = new_dist;
        found_nn = true;
      }
    }
    return found_nn;
  }

  // This allows us to use the overloaded function in the base class (passing in
  // a vector of queries instead of just one).
  using Matcher<TDescriptor, Metric>::NearestNeighbor;

  // Search for the k nearest neighbors of a single queries.
  bool KNearestNeighbors(const TDescriptor& query,
                         int k_nn,
                         std::vector<int>* knn_index,
                         std::vector<DistanceType>* knn_distance) {
    CHECK_GT(descriptors_.size(), k_nn) << "requesting more nearest neighbors "
                                        << "than elements in the search space!";
    Metric metric;
    std::priority_queue<std::pair<int, DistanceType>,
                        std::vector<std::pair<int, DistanceType> >,
                        OrderByDistance> min_heap;
    // Load up the first k elements as default values.
    for (int i = 0; i < k_nn; i++) {
      DistanceType distance = metric(query.Data(),
                                     descriptors_[i]->Data(),
                                     query.Dimensions());
      min_heap.push(std::make_pair(i, distance));
    }

    // Search the rest of the descriptors, keeping at most k_nn elements in the
    // min heap.
    for (int i = k_nn; i < descriptors_.size(); i++) {
      DistanceType distance = metric(query.Data(),
                                     descriptors_[i]->Data(),
                                     query.Dimensions());
      if (distance < min_heap.top().second) {
        min_heap.pop();
        min_heap.push(std::make_pair(i, distance));
      }
    }

    // Resize for faster allocation.
    knn_index->resize(k_nn);
    knn_distance->resize(k_nn);
    // Perform in reverse order so that the NN is first in the result.
    for (int i = k_nn - 1; i >= 0; i--) {
      std::pair<int, DistanceType> knn_pair = min_heap.top();
      min_heap.pop();
      (*knn_index)[i] = knn_pair.first;
      (*knn_distance)[i] = knn_pair.second;
    }

    return true;
  }

  // This allows us to use the overloaded function in the base class (passing in
  // a vector of queries instead of just one).
  using Matcher<TDescriptor, Metric>::KNearestNeighbors;

 private:
  struct OrderByDistance {
    bool operator ()(const std::pair<int, DistanceType>& lhs,
                     const std::pair<int, DistanceType>& rhs) const {
      return lhs.second < rhs.second;
    }
  };

  std::vector<TDescriptor*> descriptors_;
  DISALLOW_COPY_AND_ASSIGN(BruteForceMatcher);
};
}  // namespace theia
#endif  // VISION_MATCHING_BRUTE_FORCE_MATCHER_H_
