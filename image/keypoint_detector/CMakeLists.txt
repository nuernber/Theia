# See CMakeLists.txt in base dir for macro definition

# Interface for keypoint detectors.
#HEADER_LIBRARY(keypoint_detector keypoint_detector.h image/keypoint_detector/keypoint_proto)

# Keypoint protocol buffer
#PROTO_LIBRARY(keypoint_proto keypoint.proto)

IF (${BUILD_TESTING})
  #GTEST(fast_test)
ENDIF (${BUILD_TESTING})
