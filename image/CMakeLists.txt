# See CMakeLists.txt in base dir for macro definition

# Main Image class.
HEADER_LIBRARY(image image.h)

# Image Canvas class for drawing on images.
CC_LIBRARY(image_canvas image_canvas.cc 
  image/image
  image/keypoint_detector/keypoint
  util/random)

IF (${BUILD_TESTING})
  GTEST(image_test
    util/random)
ENDIF (${BUILD_TESTING})

ADD_SUBDIRECTORY(descriptor)
ADD_SUBDIRECTORY(keypoint_detector)