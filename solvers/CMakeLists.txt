# See CMakeLists.txt in base dir for macro definition

# Tests:
IF (${BUILD_TESTING})
  GTEST(arrsac_test)
  GTEST(mlesac_test)
  GTEST(ransac_test)
  GTEST(sprt_test)
ENDIF (${BUILD_TESTING})
