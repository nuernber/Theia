# See CMakeLists.txt in base dir for macro definition

# Perspective Three Point Algorithm
CC_LIBRARY(triangulation triangulation.cc)

# Tests
IF (${BUILD_TESTING})
  GTEST(triangulation_test
    vision/pose/util
    vision/triangulation/triangulation)
ENDIF (${BUILD_TESTING})
