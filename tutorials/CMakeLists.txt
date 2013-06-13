CC_BINARY(extract_sift_descriptors
  image/image
  image/image_canvas
  image/descriptor/sift_descriptor
  image/keypoint_detector/keypoint
  image/keypoint_detector/sift_detector)

CC_BINARY(extract_freak_descriptors
  image/image
  image/image_canvas
  image/descriptor/freak_descriptor
  image/keypoint_detector/keypoint
  image/keypoint_detector/brisk_detector)

CC_BINARY(match_brisk_descriptors
  image/image
  image/image_canvas
  image/descriptor/brisk_descriptor
  image/keypoint_detector/keypoint
  image/keypoint_detector/brisk_detector
  vision/matching/brute_force_matcher)

CC_BINARY(match_freak_descriptors
  image/image
  image/image_canvas
  image/descriptor/freak_descriptor
  image/keypoint_detector/keypoint
  image/keypoint_detector/brisk_detector
  vision/matching/brute_force_matcher)