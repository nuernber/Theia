# See CMakeLists.txt in base dir for macro definition

# Base descriptor classes.
HEADER_LIBRARY(descriptor descriptor.h)

# Descriptor protocol buffer.
PROTO_LIBRARY(descriptor_proto descriptor.proto image/keypoint_detector/keypoint_proto)

# Base class for descriptor extraction.
HEADER_LIBRARY(descriptor_extractor descriptor_extractor.h)

# Patch Descriptor
#CC_LIBRARY(harris_detector harris_detector.cc image/keypoint_detector/keypoint_detector image/keypoint_detector/keypoint_proto image/image)

IF (${BUILD_TESTING})
  #GTEST(harris_detector_test image/keypoint_detector/harris_detector)
ENDIF (${BUILD_TESTING})
