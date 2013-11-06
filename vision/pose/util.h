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

#ifndef VISION_POSE_UTIL_H_
#define VISION_POSE_UTIL_H_

#include <Eigen/Core>
#include <vector>

namespace theia {

// Fills the projection_matrix output variable with a projection matrix composed
// from the input parameters.
void ComposeProjectionMatrix(const double focal_length[2],
                             const double principal_point[2],
                             const double rotation[9],
                             const double translation[3],
                             double projection_matrix[12]);

// Adds noise to the 3D point passed in.
void AddNoiseToPoint(const double noise_factor, Eigen::Vector3d* point);

// Adds noise to the ray i.e. the projection of the point.
void AddNoiseToProjection(const double noise_factor, Eigen::Vector3d* ray);

void AddGaussianNoise(const double noise_factor, Eigen::Vector3d* ray);

// Creates points that are randomly distributed within a viewing frustum.
void CreateRandomPointsInFrustum(const double near_plane_width,
                                 const double near_plane_height,
                                 const double near_plane_depth,
                                 const double far_plane_depth,
                                 const int num_points,
                                 std::vector<Eigen::Vector3d>* random_points);

// Calculates Sampson distance for two correspondances and an essential or
// fundamental matrix by eq. 11.9 in Hartley and Zisserman.
double SampsonDistance(const Eigen::Matrix3d& F, const Eigen::Vector3d& x,
                       const Eigen::Vector3d& y);

// Returns the cross product matrix of a vector: if cross_vec = [x y z] then
//                        [ 0  -z   y]
// cross product matrix = [ z   0  -y]
//                        [-y   x   0]
Eigen::Matrix3d CrossProductMatrix(const Eigen::Vector3d& cross_vec);

}  // namespace theia

#endif  // VISION_POSE_UTIL_H_
