# See CMakeLists.txt in base dir for macro definitions

# Ransac target.
HEADER_LIBRARY(ransac ransac.h)

# Tests:
IF (${BUILD_TESTING})
  GTEST(ransac_test ransac)
ENDIF (${BUILD_TESTING})
