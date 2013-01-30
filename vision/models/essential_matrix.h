#ifndef VISION_MODELS_ESSENTIAL_MATRIX_H_
#define VISION_MODELS_ESSENTIAL_MATRIX_H_

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

  const Eigen::Matrix3d& GetMatrix() {
    return essential_mat_;
  }

  // Extract R and T from the essential matrix. Perform this using an
  // optimization to extract the SVD for this 3x3 matrix.
  void Decompose(double rotation[3][3],
                 double translation[3]);
  
  friend std::ostream& operator <<(std::ostream& os,
                                   const EssentialMatrix& mat);
  
 private:
  Eigen::Matrix3d essential_mat_;
};
}  // namespace models
}  // namespace vision


#endif  // VISION_MODELS_ESSENTIAL_MATRIX_H_
