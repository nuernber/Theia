#include "vision/pose/five_point_relative_pose.h"
#include "gtest/gtest.h"
#include <Eigen/Core>
#include <iostream>

namespace vision {
namespace pose {
using Eigen::Matrix3d;

TEST(FivePointRelativePose, Sanity) {
  // Ground truth essential matrix.
  Matrix3d essential_mat;
  essential_mat << 21.36410238362200, -6.45974435705903, -12.57571428668080,
      -7.57644257286238, -20.76739671331773, 28.55199769513539,
      23.52168039400320, -21.32072430358359, 1.00000000000000;

  // corresponding points
  double m1[5][2] = {{-0.06450996083506, 0.13271495297568},
                     {0.12991134106070, 0.25636930852309},
                     {-0.01417169260989, -0.00201819071777},
                     {0.02077599827229, 0.14974189941659},
                     {0.15970376410206, 0.17782283760921}};
  double m2[5][2] = {{0.01991821260977, 0.13855920297370},
                     {0.29792009989498, 0.21684093037950},
                     {0.14221131487001, 0.03902000959363},
                     {0.09012597508721, 0.11407993279726},
                     {0.15373963107017, 0.02622710108650}};
  double estimated_essential_mat[11][3][3];
  int num_solutions = FivePointRelativePose(m1, m2, estimated_essential_mat);
  std::cout << "Num solutions found = " << num_solutions << std::endl;

}

}  // namespace pose
}  // namespace vision
