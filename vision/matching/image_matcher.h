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

#ifndef VISION_MATCHING_IMAGE_MATCHER_H_
#define VISION_MATCHING_IMAGE_MATCHER_H_

#include <unordered_map>
#include <vector>

#include "util/map_util.h"
#include "util/util.h"
#include "vision/matching/brute_force_matcher.h"

namespace theia {
template<class T>
struct FeatureMatch {
  FeatureMatch(int f1_ind, int f2_ind, T dist)
      : feature1_ind(f1_ind), feature2_ind(f2_ind), distance(dist) {}
  // Index of the feature in the first image.
  int feature1_ind;
  // Index of the feature in the second image.
  int feature2_ind;
  // Distance between the two features.
  T distance;
};

// Generic class for matching between two images.
template<class Matcher>
class ImageMatcher {
 public:
  typedef typename Matcher::DistanceType DistanceType;
  typedef typename Matcher::DescriptorType DescriptorType;

  ImageMatcher() {}
  virtual ~ImageMatcher() {}

  // Return the all NN matches that pass the threshold.
  virtual bool Match(const std::vector<DescriptorType*>& desc_1,
                     const std::vector<DescriptorType*>& desc_2,
                     std::vector<FeatureMatch<DistanceType> >* matches,
                     DistanceType threshold = 0);

  // Return all the NN matches that pass the threshold and pass the ratio test
  // (distance of 1st NN / distance of 2nd NN < ratio).
  virtual bool MatchDistanceRatio(
      const std::vector<DescriptorType*>& desc_1,
      const std::vector<DescriptorType*>& desc_2,
      std::vector<FeatureMatch<DistanceType> >* matches,
      double ratio,
      DistanceType threshold = 0);

  // Only returns matches that are mutual nearest neighbors (i.e. if a
  // descriptor x in image 1 has a NN y in image 2, then y's NN is x).
  virtual bool MatchSymmetric(const std::vector<DescriptorType*>& desc_1,
                              const std::vector<DescriptorType*>& desc_2,
                              std::vector<FeatureMatch<DistanceType> >* matches,
                              DistanceType threshold = 0);

  // Returns matches that pass the symmetric test and the distance ratio test.
  virtual bool MatchSymmetricAndDistanceRatio(
      const std::vector<DescriptorType*>& desc_1,
      const std::vector<DescriptorType*>& desc_2,
      std::vector<FeatureMatch<DistanceType> >* matches,
      double ratio,
      DistanceType threshold = 0);

 private:
  void IntersectMatches(const std::vector<FeatureMatch<DistanceType> >& match1,
                        const std::vector<FeatureMatch<DistanceType> >& match2,
                        std::vector<FeatureMatch<DistanceType> >* output_match);

  DISALLOW_COPY_AND_ASSIGN(ImageMatcher);
};

template <class T, class D>
class BruteForceImageMatcher : public ImageMatcher<BruteForceMatcher<T, D> > {
};

// ---------------------- Implementation ------------------------ //
template<class Matcher>
bool ImageMatcher<Matcher>::Match(
    const std::vector<DescriptorType*>& desc_1,
    const std::vector<DescriptorType*>& desc_2,
    std::vector<FeatureMatch<DistanceType> >* matches,
    DistanceType threshold) {
  Matcher matcher;
  matcher.Build(desc_2);

  // Find matches and move the data into the FeatureMatch output variable.
  matches->reserve(desc_1.size());
  for (int i = 0; i < desc_1.size(); i++) {
    int index;
    DistanceType distance;
    // If matching is successful and passes the threshold.
    if (matcher.NearestNeighbor(*desc_1[i],
                                &index,
                                &distance,
                                threshold)) {
      matches->push_back(FeatureMatch<DistanceType>(i,
                                                    index,
                                                    distance));
    }
  }
  return true;
}

template<class Matcher>
bool ImageMatcher<Matcher>::MatchDistanceRatio(
    const std::vector<DescriptorType*>& desc_1,
    const std::vector<DescriptorType*>& desc_2,
    std::vector<FeatureMatch<DistanceType> >* matches,
    double ratio,
    DistanceType threshold) {
  Matcher matcher;
  matcher.Build(desc_2);

  // Find matches and move the data into the FeatureMatch output variable.
  matches->reserve(desc_1.size());
  for (int i = 0; i < desc_1.size(); i++) {
    LOG(INFO) << "Matching descriptor " << i;
    std::vector<int> index;
    std::vector<DistanceType> distance;
    // If matching is successful.
    if (matcher.KNearestNeighbors(*desc_1[i],
                                  2,
                                  &index,
                                  &distance)) {
      // If the distance passes the ratio test and the distance is less then the
      // threshold (if one is set).
      LOG(INFO) << "inside if";
      if (distance[0]/distance[1] < ratio &&
          (threshold <= 0 || distance[0] < threshold)) {
        LOG(INFO) << "inside second if";
        matches->push_back(FeatureMatch<DistanceType>(i,
                                                      index[0],
                                                      distance[0]));
      }
    }
  }
  return true;
}

template<class Matcher>
bool ImageMatcher<Matcher>::MatchSymmetric(
    const std::vector<DescriptorType*>& desc_1,
    const std::vector<DescriptorType*>& desc_2,
    std::vector<FeatureMatch<DistanceType> >* matches,
    DistanceType threshold) {
  std::vector<FeatureMatch<DistanceType> > forward_matches;
  std::vector<FeatureMatch<DistanceType> > backward_matches;
  // Get matches from image 1 to image 2.
  Match(desc_1, desc_2, &forward_matches, threshold);
  // Get matches from image 2 to image 1.
  Match(desc_2, desc_1, &backward_matches, threshold);
  // Interesect the two sets.
  IntersectMatches(forward_matches, backward_matches, matches);
  return true;
}

template<class Matcher>
bool ImageMatcher<Matcher>::MatchSymmetricAndDistanceRatio(
    const std::vector<DescriptorType*>& desc_1,
    const std::vector<DescriptorType*>& desc_2,
    std::vector<FeatureMatch<DistanceType> >* matches,
    double ratio,
    DistanceType threshold) {
  std::vector<FeatureMatch<DistanceType> > forward_matches;
  std::vector<FeatureMatch<DistanceType> > backward_matches;
  // Get all matches from image 1 to 2 that pass the ratio test.
  MatchDistanceRatio(desc_1, desc_2, &forward_matches, ratio);
  // Get all matches from image 2 to 1 that pass the ratio test.
  MatchDistanceRatio(desc_2, desc_1, &backward_matches, ratio);
  // Intersect the two sets.
  IntersectMatches(forward_matches, backward_matches, matches);

  // Remove all matches that do not pass the threshold test.
  auto match_iterator = matches->begin();
  while (match_iterator != matches->end()) {
    if (match_iterator->distance >= threshold) {
      match_iterator = matches->erase(match_iterator);
    } else {
      ++match_iterator;
    }
  }
  return true;
}

template<class Matcher>
void ImageMatcher<Matcher>::IntersectMatches(
    const std::vector<FeatureMatch<DistanceType> >& match1,
    const std::vector<FeatureMatch<DistanceType> >& match2,
    std::vector<FeatureMatch<DistanceType> >* output_match) {
  std::unordered_map<int, int> index_map;
  index_map.reserve(match2.size());
  // Add all feature2 -> feature1 matches to the map.
  for (const FeatureMatch<DistanceType>& feature_match : match2) {
    InsertOrDie(&index_map,
                feature_match.feature1_ind,
                feature_match.feature2_ind);
  }

  // Search the map for feature1 -> feature2 matches that are also present in
  // the feature2 -> feature1 matches.
  output_match->reserve(match1.size());
  for (const FeatureMatch<DistanceType>& feature_match : match1) {
    int feature1_index = FindWithDefault(index_map,
                                         feature_match.feature2_ind,
                                         -1);
    if (feature1_index == feature_match.feature1_ind) {
      output_match->push_back(feature_match);
    }
  }
}
}  // namespace theia
#endif  // VISION_MATCHING_IMAGE_MATCHER_H_
