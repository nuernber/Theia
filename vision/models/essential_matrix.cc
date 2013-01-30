#include "vision/models/essential_matrix.h"

#include <Eigen/Dense>
#include <ctime>
#include <iostream>

namespace vision {
namespace models {
using Eigen::Matrix3d;

EssentialMatrix::EssentialMatrix(const double data[3][3]) {
  essential_mat_ << data[0][0], data[0][1], data[0][2],
      data[1][0], data[1][1], data[1][2],
      data[2][0], data[2][1], data[2][2];
}

void EssentialMatrix::Decompose(double rotation[3][3],
                                double translation[3]) {
  using Eigen::Vector3d;

  Matrix3d d;
  d << 0, 1, 0,
      -1, 0, 0,
      0, 0, 1;

  clock_t t = clock();
  Eigen::JacobiSVD<Matrix3d> svd =
      essential_mat_.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix3d u_me = svd.matrixU();
  Matrix3d v_me = svd.matrixV();
  t = clock();
  std::cout << "their u = \n" << u_me << std::endl;
  std::cout << "their v = \n" << v_me << std::endl;
  std::cout << "r = " << u_me*d*(v_me.transpose()) << std::endl;
  t = clock() - t;
  printf("Eigen Version took me %d clicks (%f seconds).\n",
         t,
         ((float)t)/CLOCKS_PER_SEC);

  //
  Vector3d large_a = essential_mat_.row(0);
  Vector3d large_b = essential_mat_.row(1);
  if (large_a.cross(large_b).squaredNorm() <
      essential_mat_.row(0).cross(essential_mat_.row(2)).squaredNorm()) {
    large_b = essential_mat_.row(2);
  }
  if (large_a.cross(large_b).squaredNorm() <
      essential_mat_.row(1).cross(essential_mat_.row(2)).squaredNorm()) {
    large_a = essential_mat_.row(1);
    large_b = essential_mat_.row(2);
  }

  Matrix3d v;
  v.col(2) = (large_a.cross(large_b)).normalized();
  v.col(0) = large_a.normalized();
  v.col(1) = v.col(2).cross(v.col(0));
  Matrix3d u;
  u.col(0) = (essential_mat_*v.col(0)).normalized();
  u.col(1) = (essential_mat_*v.col(1)).normalized();
  u.col(2) = u.col(0).cross(u.col(1));
  t = clock() - t;
  LOG(INFO) << "my u = \n" << u;
  LOG(INFO) << "my v = \n" << v;
  LOG(INFO) << "r = " << u*d*(v.transpose());

  printf("My Version took me %d clicks (%f seconds).\n",
         t,
         ((float)t)/CLOCKS_PER_SEC);

}

std::ostream& operator <<(std::ostream& os,
                          const EssentialMatrix& mat) {
  os << mat.essential_mat_;
  return os;
}

}  // namespace models
}  // namespace vision
