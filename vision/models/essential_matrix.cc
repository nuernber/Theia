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
#include <algorithm>
#include <Eigen/Dense>
#include <glog/logging.h>

namespace vision {
namespace models {
using Eigen::Matrix3d;
using Eigen::Vector3d;

EssentialMatrix::EssentialMatrix(const double data[3][3]) {
  essential_mat_ << data[0][0], data[0][1], data[0][2],
      data[1][0], data[1][1], data[1][2],
      data[2][0], data[2][1], data[2][2];
}

void EssentialMatrix::Decompose(double rotation[4][3][3],
                                double translation[4][3]) {
  Matrix3d d;
  d << 0, 1, 0,
      -1, 0, 0,
      0, 0, 1;
  Matrix3d diag;
  diag << 1, 0, 0,
      0, 1, 0,
      0, 0, 0;

  const Vector3d& ea = essential_mat_.row(0);
  const Vector3d& eb = essential_mat_.row(1);
  const Vector3d& ec = essential_mat_.row(2);

  // Generate cross products.
  Matrix3d cross_products;
  cross_products << ea.cross(eb), ea.cross(ec), eb.cross(ec);
  VLOG(0) << "cross term =\n" << ea.cross(eb);
  VLOG(0) << "cross_products = \n" << cross_products;

  // Choose the cross product with the largest norm (for numerical accuracy).
  const Vector3d cf_scales(cross_products.col(0).squaredNorm(),
                           cross_products.col(1).squaredNorm(),
                           cross_products.col(2).squaredNorm());
  VLOG(0) << "cf scales =\n" << cf_scales;
  int max_index;
  double max_cf = cf_scales.maxCoeff(&max_index);
  VLOG(0) << "max index = " << max_index;

  // For index 0, 1, we want ea and for index 2 we want eb.
  int max_e_index = max_index/2;
  VLOG(0) << "max e index = " << max_e_index;

  // Construct v of the SVD.
  Matrix3d v = Matrix3d::Zero();
  v.col(2) = cross_products.col(max_index).normalized();
  v.col(0) = essential_mat_.row(max_e_index).normalized();
  v.col(1) = v.col(2).cross(v.col(0));

  // Construct U of the SVD.
  Matrix3d u = Matrix3d::Zero();
  u.col(0) = (essential_mat_*v.col(0)).normalized();
  u.col(1) = (essential_mat_*v.col(1)).normalized();
  u.col(2) = u.col(0).cross(u.col(1));
  
  VLOG(0) << "U = \n" << u;
  VLOG(0) << "V = \n" << v;
  VLOG(0) << "E = \n" << u*diag*(v.transpose());

  // Possible rotation configurations.
  const Matrix3d ra = u*d*v.transpose();
  const Matrix3d rb = u*d.transpose()*v.transpose();

  // Scale t to be proper magnitude. Scale factor is derived from the fact that
  // U*diag*V^t = E. We simply choose to scale it such that the last terms will
  // be equal.
  const double scale = essential_mat_(2,2)/(u(2, 0)*v(2, 0) + u(2, 1)*v(2, 1));
  const Vector3d t = scale*u.col(2);
  const Vector3d t_neg = -t;
  VLOG(0) << "Ra = \n" << ra;
  VLOG(0) << "Rb = \n" << rb;
  VLOG(0) << "t = \n" << t;

  // Copy the 4 possible decompositions into the output arrays.
  std::copy(ra.data(), ra.data() + ra.size(),
            reinterpret_cast<double*>(rotation[0]));
  std::copy(t.data(), t.data() + t.size(), translation[0]);
  std::copy(ra.data(), ra.data() + ra.size(),
            reinterpret_cast<double*>(rotation[1]));
  std::copy(t_neg.data(), t_neg.data() + t_neg.size(), translation[1]);
  std::copy(rb.data(), rb.data() + rb.size(),
            reinterpret_cast<double*>(rotation[2]));
  std::copy(t.data(), t.data() + t.size(), translation[2]);
  std::copy(rb.data(), rb.data() + rb.size(),
            reinterpret_cast<double*>(rotation[3]));
  std::copy(t_neg.data(), t_neg.data() + t_neg.size(), translation[3]);
}

std::ostream& operator <<(std::ostream& os,
                          const EssentialMatrix& mat) {
  os << mat.essential_mat_;
  return os;
}

}  // namespace models
}  // namespace vision
