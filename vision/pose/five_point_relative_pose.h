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

#ifndef VISION_POSE_FIVE_POINT_RELATIVE_POSE_H_
#define VISION_POSE_FIVE_POINT_RELATIVE_POSE_H_

#include <vector>
#include "vision/models/essential_matrix.h"

namespace vision {
namespace pose {
// Computes the relative pose between two cameras using 5 corresponding
// points. Algorithm is implemented based on "An Efficient Solution to the
// Five-Point Relative Pose Problem" by Nister.
//
// Params:
//   image1_points: Location of features on the image plane (x[i][*] = i-th
//     image point)
//   image2_points: Location of features on the image plane (x[i][*] = i-th
//     image point)
// Return: essential_matrix: Output all solutions of the 5 point algorithm.
std::vector<vision::models::EssentialMatrix> FivePointRelativePose(
    const double image1_points[3][2],
    const double image2_points[3][2]);
}  // pose
}  // vision

#endif  // VISION_POSE_FIVE_POINT_RELATIVE_POSE_H_
