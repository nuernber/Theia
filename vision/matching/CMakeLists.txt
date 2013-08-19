# Matcher interface.
HEADER_LIBRARY(matcher matcher.h)

# Brute force matching.
HEADER_LIBRARY(brute_force_matcher brute_force_matcher.h 
  vision/matching/matcher)

# Image Matching.
HEADER_LIBRARY(image_matcher image_matcher.h)

# Distances (L2, hamming, etc.).
HEADER_LIBRARY(distance distance.h
  image/descriptor/descriptor)

if (${BUILD_TESTING})
  # L2 and hamming distance tests.
  GTEST(distance_test
    vision/matching/distance
    util/random)
    
ENDIF (${BUILD_TESTING})