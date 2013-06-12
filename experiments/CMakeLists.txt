GTEST(ncc_test
  util/random)

CC_BINARY(sift_pairwise_matches
  image/image
  image/descriptor/sift_descriptor
  image/keypoint_detector/keypoint
  image/keypoint_detector/sift_detector
  vision/matching/brute_force_matcher)