# See CMakeLists.txt in base dir for macro definition

# Perspective Three Point Algorithm
CC_LIBRARY(perspective_three_point perspective_three_point.cc  
  math/closed_form_polynomial_solver)

# 5 Point relative pose
CC_LIBRARY(five_point_relative_pose five_point_relative_pose.cc 
  vision/models/essential_matrix)

# Tests:
IF (${BUILD_TESTING})
  GTEST(perspective_three_point_test 
    vision/pose/perspective_three_point)
  GTEST(five_point_relative_pose_test 
    vision/pose/five_point_relative_pose 
    vision/models/essential_matrix)
ENDIF (${BUILD_TESTING})
