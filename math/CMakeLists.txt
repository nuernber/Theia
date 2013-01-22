# See CMakeLists.txt in base dir for macro definition

# Closed form polynomial solver
CC_LIBRARY(closed_form_polynomial_solver closed_form_polynomial_solver.cc)

# Generic Polynomial Class
CC_LIBRARY(polynomial polynomial.cc)

# Tests:
IF (${BUILD_TESTING})
  GTEST(closed_form_polynomial_solver_test math/closed_form_polynomial_solver)
  GTEST(polynomial_test math/polynomial)
ENDIF (${BUILD_TESTING})

ADD_SUBDIRECTORY(matrix)
