# See CMakeLists.txt in base dir for macro definition

# Essential Matrix
CC_LIBRARY(essential_matrix essential_matrix.cc)

# Tests:
IF (${BUILD_TESTING})
  GTEST(essential_matrix_test vision/models/essential_matrix)
ENDIF (${BUILD_TESTING})
