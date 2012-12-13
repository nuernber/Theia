# See CMakeLists.txt in base dir for macro definition

# Tests:
IF (${BUILD_TESTING})
  GTEST(ransac_test)
  GTEST(sprt_test)
  GTEST(arrsac_test)
ENDIF (${BUILD_TESTING})
