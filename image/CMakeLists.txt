# See CMakeLists.txt in base dir for macro definition

# Main Image class


IF (${BUILD_TESTING})
  GTEST(image_test)
ENDIF (${BUILD_TESTING})