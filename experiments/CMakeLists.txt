GTEST(ncc_test
  util/random)

CC_BINARY(sift_pairwise_matches
  image/image
  image/image_canvas
  image/descriptors/sift_descriptors
  image/keypoint_detector/keypoint
  image/keypoint_detector/sift_detector
  vision/matching/brute_force_matcher)