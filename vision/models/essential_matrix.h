#ifndef VISION_MODELS_ESSENTIAL_MATRIX_
#define VISION_MODELS_ESSENTIAL_MATRIX_

#include <Eigen/Core>
#include <ostream>

namespace vision {
namespace models {
class EssentialMatrix {
 public:
  EssentialMatrix() {}
  EssentialMatrix(const Eigen::Matrix3d& essential_mat)
      : essential_mat_(essential_mat) {}
  EssentialMatrix(const double data[3][3]);
  ~EssentialMatrix() {}

  const Eigen::Matrix3d& GetMatrix() {return essential_mat_;};
  
  friend std::ostream& operator <<(std::ostream& os, const EssentialMatrix& mat);

  private:
  Eigen::Matrix3d essential_mat_;

};
}  // namespace models
}  // namespace vision


#endif  // VISION_MODELS_ESSENTIAL_MATRIX_
