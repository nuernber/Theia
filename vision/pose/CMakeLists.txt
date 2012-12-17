# See CMakeLists.txt in base dir for macro definition

# Perspective Three Point Algorithm
CC_LIBRARY(perspective_three_point perspective_three_point.cc  math/closed_form_polynomial_solver vision/transformation/iterative_closest_point)

# Tests:
IF (${BUILD_TESTING})
  GTEST(perspective_three_point_test vision/pose/perspective_three_point)
ENDIF (${BUILD_TESTING})
