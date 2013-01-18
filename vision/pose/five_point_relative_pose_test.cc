#include "vision/pose/five_point_relative_pose.h"

#include "vision/models/essential_matrix.h"
#include "gtest/gtest.h"
#include <Eigen/Core>
#include <vector>

#include <iostream>
#include <ctime>

namespace vision {
namespace pose {
using Eigen::Matrix3d;
using vision::models::EssentialMatrix;

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

  clock_t t = clock();
  std::vector<EssentialMatrix> essential_matrices =
      FivePointRelativePose(m1, m2);
  t = clock() - t;
  printf ("My version took me %d clicks (%f seconds).\n",t,((float)t)/CLOCKS_PER_SEC);

  std::cout << "Num solutions found = " << essential_matrices.size() << std::endl;
  for (int i = 0; i < essential_matrices.size(); i++) {
    const Matrix3d& essential_matrix = essential_matrices[i].GetMatrix();
    std::cout << "Essential Matrix =\n" << essential_matrix << std::endl;
    // Use Map.
    Eigen::Vector3d u(m1[i][0], m1[i][1], 1.0);
    Eigen::Vector3d u_prime(m2[i][0], m2[i][1], 1.0);
    double epipolar_constraint = (u_prime.transpose())*essential_matrix*u;
    std::cout << "Epipolar constraint = " << epipolar_constraint << std::endl;
  }
}

}  // namespace pose
}  // namespace vision
