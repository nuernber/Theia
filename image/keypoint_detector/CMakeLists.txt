# See CMakeLists.txt in base dir for macro definitions.
# Keypoint class.
CC_LIBRARY(keypoint keypoint.cc image/keypoint_detector/keypoint_proto)

# Interface for keypoint detectors.
HEADER_LIBRARY(keypoint_detector keypoint_detector.h)

# Keypoint protocol buffer.
PROTO_LIBRARY(keypoint_proto keypoint.proto)

# Fast Keypoint detector.
CC_LIBRARY(fast_detector fast_detector.cc
  image/keypoint_detector/keypoint_detector
  image/keypoint_detector/keypoint
  image/image)

# Harris Corner detector.
CC_LIBRARY(harris_detector harris_detector.cc
  image/keypoint_detector/keypoint_detector
  image/keypoint_detector/keypoint
  image/image)

# SIFT Detector.
CC_LIBRARY(sift_detector sift_detector.cc
  image/keypoint_detector/keypoint_detector
  image/keypoint_detector/keypoint
  image/image
  vlfeat)

IF (${BUILD_TESTING})
  GTEST(fast_detector_test image/keypoint_detector/fast_detector)
  GTEST(harris_detector_test image/keypoint_detector/harris_detector)
ENDIF (${BUILD_TESTING})
