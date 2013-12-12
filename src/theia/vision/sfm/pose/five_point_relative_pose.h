// Copyright (C) 2013 The Regents of the University of California (Regents).
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
//     * Neither the name of The Regents or University of California nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Please contact the author of this library if you have any questions.
// Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

#ifndef THEIA_VISION_SFM_POSE_FIVE_POINT_RELATIVE_POSE_H_
#define THEIA_VISION_SFM_POSE_FIVE_POINT_RELATIVE_POSE_H_

#include <Eigen/Core>
#include <vector>

namespace theia {

// Computes the relative pose between two cameras using 5 corresponding
// points. Algorithm is implemented based on "An Efficient Solution to the
// Five-Point Relative Pose Problem" by Nister.
//
// Params:
//   image1_points: Location of features on the image plane (x[i * 5] = i-th
//     image point)
//   image2_points: Location of features on the image plane of image 2
//     (x[i * 5] = i-th image point)
// Return: essential_matrix: Output all solutions of the 5 point algorithm.
int FivePointRelativePose(const double image1_points[3 * 5],
                          const double image2_points[3 * 5],
                          double rotation[9 * 10],
                          double translation[3 * 10]);

// Same as above, but with the Eigen interface. Returns true if more than one
// solution was found.
bool FivePointRelativePose(const Eigen::Vector3d image1_points[5],
                           const Eigen::Vector3d image2_points[5],
                           std::vector<Eigen::Matrix3d>* rotation,
                           std::vector<Eigen::Vector3d>* translation);
}  // namespace theia

#endif  // THEIA_VISION_SFM_POSE_FIVE_POINT_RELATIVE_POSE_H_
