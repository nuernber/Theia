#See CMakeLists.txt in base dir for macro definition

#Tests:
IF($ {
  BUILD_TESTING
})
GTEST(gauss_jordan_test)
ENDIF($ {
  BUILD_TESTING
})
