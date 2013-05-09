# See CMakeLists.txt in base dir for macro definition

# Base descriptor classes.
HEADER_LIBRARY(descriptor descriptor.h)

# Descriptor protocol buffer.
PROTO_LIBRARY(descriptor_proto descriptor.proto image/keypoint_detector/keypoint_proto)

# Base class for descriptor extraction.
HEADER_LIBRARY(descriptor_extractor descriptor_extractor.h)

# Patch Descriptor
HEADER_LIBRARY(patch_descriptor patch_descriptor.h image/descriptor/descriptor image/descriptor/descriptor_proto image/image)

IF (${BUILD_TESTING})
  GTEST(patch_descriptor_test image/descriptor/patch_descriptor image/descriptor/descriptor_proto image/image image/keypoint_detector/fast_detector)
ENDIF (${BUILD_TESTING})
