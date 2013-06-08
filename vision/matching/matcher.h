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

#ifndef VISION_MATCHING_MATCHER_H_
#define VISION_MATCHING_MATCHER_H_

#include <vector>

namespace theia {
// Template Descriptor can be any type of descriptor, and Metric can be any of
// the distance metrics defined in distance.h
template <class TemplateDescriptor, class Metric>
class Matcher {
 public:
  typedef typename Metric::ResultType TDistanceType;

  Matcher() {}
  virtual ~Matcher() {}

  // Build an index or other data structures needed to perform the search.
  virtual bool Build(const std::vector<TemplateDescriptor*>& descriptors) = 0;

  // Search for the sole nearest neighbor for a single query.
  virtual bool NearestNeighbor(const TemplateDescriptor& query,
                               int* neighbor_index,
                               TDistanceType* distance) = 0;

  // Search for the sole nearest neighbor for a multiple queries.
  virtual bool NearestNeighbor(
      const std::vector<TemplateDescriptor*>& queries,
      std::vector<int>* neighbor_indices,
      std::vector<TDistanceType>* distances) {
    neighbor_indices->resize(queries.size());
    distances->resize(queries.size());
    for (int i = 0; i < queries.size(); i++) {
      NearestNeighbor(*queries[i], &((*neighbor_indices)[i]), &((*distances)[i]));
    }
    return true;
  }

  // Search for the k nearest neighbors of a single queries.
  virtual bool KNearestNeighbors(const TemplateDescriptor& query,
                                 int k_nn,
                                 std::vector<int>* knn_index,
                                 std::vector<TDistanceType>* knn_distance) = 0;

  // Search for the k nearest neighbors of a multiple queries.
  virtual bool KNearestNeighbors(
      const std::vector<TemplateDescriptor*>& queries,
      int k_nn,
      std::vector<std::vector<int> >* knn_indices,
      std::vector<std::vector<TDistanceType> >* knn_distances) {

  }
};
}  // namespace theia
#endif  // VISION_MATCHING_MATCHER_H_
