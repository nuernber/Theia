#include "vision/models/essential_matrix.h"

#include <Eigen/Core>

namespace vision {
namespace models {

EssentialMatrix::EssentialMatrix(const double data[3][3]) {
  essential_mat_ << data[0][0], data[0][1], data[0][2],
      data[1][0], data[1][1], data[1][2],
      data[2][0], data[2][1], data[2][2];
}
std::ostream& operator <<(std::ostream& os, const EssentialMatrix& mat) {
  os << mat.essential_mat_;
  return os;
}

}  // namespace models
}  // namespace vision
