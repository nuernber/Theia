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

# Agast (improved Fast method) detector.
CC_LIBRARY(agast_detector agast_detector.cc
  image/keypoint_detector/keypoint_detector
  image/keypoint_detector/keypoint
  image/image
  agast)

# Helper library for BRISK detector.
CC_LIBRARY(brisk_impl brisk_impl.cc
 image/keypoint_detector/keypoint
 image/image
 agast)

# BRIEF detector.
CC_LIBRARY(brisk_detector brisk_detector.cc
  image/keypoint_detector/brisk_impl
  image/keypoint_detector/keypoint_detector
  image/keypoint_detector/keypoint
  image/image
  agast)

IF (${BUILD_TESTING})
  GTEST(fast_detector_test 
    image/keypoint_detector/fast_detector
    util/random)
  GTEST(harris_detector_test 
    image/keypoint_detector/harris_detector
    util/random)

  GTEST(sift_detector_test 
    image/keypoint_detector/sift_detector
    util/random)

  GTEST(agast_detector_test 
    image/keypoint_detector/agast_detector
    util/random)

  GTEST(brisk_detector_test 
    image/keypoint_detector/brisk_detector
    util/random)
  
ENDIF (${BUILD_TESTING})
