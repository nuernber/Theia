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

#include "vision/models/essential_matrix.h"

#include <Eigen/Dense>
#include <glog/logging.h>

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
  Matrix3d ra = u*d*(v.transpose());
  Matrix3d rb = u*d.transpose()*(v.transpose());
  Vector3d t(u.col(2));
}

std::ostream& operator <<(std::ostream& os,
                          const EssentialMatrix& mat) {
  os << mat.essential_mat_;
  return os;
}

}  // namespace models
}  // namespace vision
