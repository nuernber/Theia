#include "vision/models/essential_matrix.h"

#include "gtest/gtest.h"
#include <Eigen/Dense>

namespace vision {
namespace models {
using Eigen::Matrix3d;

TEST(EssentialMat, GetRandT) {
  Matrix3d rotation = Matrix3d::Random();
  Eigen::Vector3d trans = Eigen::Vector3d::Random();
  Matrix3d translation;
  translation << 0, -trans(2), trans(1),
      trans(2), 0, -trans(0),
      -trans(1), trans(0), 0;
  std::cout << "true rotation = \n" << rotation << std::endl;
  std::cout << "true translation = \n" << trans << std::endl;
  EssentialMatrix my_essential_mat(rotation*translation);
  std::cout << "Essential mat = \n" << my_essential_mat << std::endl;

  double my_rotation[3][3];
  double my_translation[3];
  my_essential_mat.GetRotationAndTranslation(my_rotation, my_translation);
}

}  // namespace models
}  // namespace vision
