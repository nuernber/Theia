# See CMakeLists.txt in base dir for macro definition

# Ransac
HEADER_LIBRARY(ransac ransac.h)

# SPRT. Confusing name, but note that this is not a test (though there is a test
# for it below)
HEADER_LIBRARY(sequential_probability_ratio_test sequential_probability_ratio_test.h)

# ARRSAC
HEADER_LIBRARY(arrsac arrsac.h solvers/sequential_probability_ratio_test)

# Tests:
IF (${BUILD_TESTING})
  GTEST(ransac_test ransac)
  GTEST(sprt_test sequential_probability_ratio_test)
  GTEST(arrsac_test arrsac)
ENDIF (${BUILD_TESTING})
