# See CMakeLists.txt in base dir for macro definition

# Besl-McKay registration of point clouds
CC_LIBRARY(align_point_clouds align_point_clouds.cc)

# Tests:
IF (${BUILD_TESTING})
  GTEST(align_point_clouds_test vision/transformation/align_point_clouds)
ENDIF (${BUILD_TESTING})
