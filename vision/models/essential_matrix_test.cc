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
