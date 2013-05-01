# See CMakeLists.txt in base dir for macro definition

# Interface for keypoint detectors.
HEADER_LIBRARY(keypoint_detector keypoint_detector.h image/keypoint_detector/keypoint_proto)

# Keypoint protocol buffer
PROTO_LIBRARY(keypoint_proto keypoint.proto)

# Fast Keypoint detector
CC_LIBRARY(fast_detector fast_detector.cc image/keypoint_detector/keypoint_detector image/image)

IF (${BUILD_TESTING})
  GTEST(fast_detector_test image/keypoint_detector/fast_detector)
ENDIF (${BUILD_TESTING})
