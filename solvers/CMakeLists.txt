# See CMakeLists.txt in base dir for macro definition

# Closed form polynomial solver
CC_LIBRARY(closed_form_polynomial_solver closed_form_polynomial_solver.cc)

# Tests:
IF (${BUILD_TESTING})
  GTEST(ransac_test)
  GTEST(sprt_test)
  GTEST(arrsac_test)
  GTEST(closed_form_polynomial_solver_test solvers/closed_form_polynomial_solver)
ENDIF (${BUILD_TESTING})
