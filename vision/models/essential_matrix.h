// Copyright (C) 2013  Chris Sweeney <cmsweeney@cs.ucsb.edu>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of the University of California, Santa Barbara nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL CHRIS SWEENEY BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#ifndef VISION_MODELS_ESSENTIAL_MATRIX_H_
#define VISION_MODELS_ESSENTIAL_MATRIX_H_

#include <Eigen/Core>
#include <ostream>

namespace vision {
namespace models {
class EssentialMatrix {
 public:
  EssentialMatrix() {}
  explicit EssentialMatrix(const Eigen::Matrix3d& essential_mat)
      : essential_mat_(essential_mat) {}
  explicit EssentialMatrix(const double data[3][3]);
  ~EssentialMatrix() {}

  const Eigen::Matrix3d& GetMatrix() {
    return essential_mat_;
  }

  // Extract R and T from the essential matrix. Perform this using an
  // optimization to extract the SVD for this 3x3 matrix. All decomposition
  // methods return R, T such that E = Tx * R where Tx is the cross product
  // matrix of T
  void Decompose(double rotation[4][3][3],
                 double translation[4][3]) const;

  // Extract R and T from the essential matrix. Find the optimal decomposition
  // by triangulating a point and ensuring the cheirality constraint (that the
  // point is in front of the camera). This assumes the point correspondance
  // passed in was a point used for the 5 point algorithm (i.e. an ideal
  // point). Implemented according to Appendix C of the Nister paper.
  void DecomposeWithIdealCorrespondence(const double image_point1[3],
                                        const double image_point2[3],
                                        double rotation[3][3],
                                        double translation[3]) const;


  friend std::ostream& operator <<(std::ostream& os,
                                   const EssentialMatrix& mat);

 private:
  Eigen::Matrix3d essential_mat_;

  // TODO(cmsweeney): Make this not private! It was moved to private because I
  // could not figure out how to make it work properly. Triangulation is
  // performed according to appendix C of the Nister "EFficient Five Point Alg."
  // paper, but it does not work consistently for non-ideal (i.e. noisy) points.
  //
  // Extract R and T from the essential matrix.  Same as above, but does not
  // require an ideal correspondance (i.e. the correspondance does NOT have to
  // be one used in the five point to generate the essential matrix).
  void DecomposeWithCorrespondence(
      int num_correspondences,
      const double image_point1[][3],
      const double image_point2[][3],
      double rotation[3][3],
      double translation[3]) const;
};
}  // namespace models
}  // namespace vision


#endif  // VISION_MODELS_ESSENTIAL_MATRIX_H_
