# See CMakeLists.txt in base dir for macro definition

# Main Image class
HEADER_LIBRARY(image image.h)

IF (${BUILD_TESTING})
  GTEST(image_test)
ENDIF (${BUILD_TESTING})

ADD_SUBDIRECTORY(descriptor)
ADD_SUBDIRECTORY(keypoint_detector)