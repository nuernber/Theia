# See CMakeLists.txt in base dir for macro definition

# Ransac
HEADER_LIBRARY(ransac ransac.h)

# SPRT
HEADER_LIBRARY(sequential_probability_ratio_test sequential_probability_ratio_test.h)

# ARRSAC
CC_LIBRARY(arrasc arrsac.cc 
  solvers/sequential_probability_ratio_test)

# Tests:
IF (${BUILD_TESTING})
  GTEST(ransac_test ransac)
ENDIF (${BUILD_TESTING})
