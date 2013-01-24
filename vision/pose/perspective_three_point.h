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

#ifndef VISION_POSE_PERSPECTIVE_THREE_POINT_H_
#define VISION_POSE_PERSPECTIVE_THREE_POINT_H_

namespace vision {
namespace pose {
// Computes camera pose using the three point algorithm and returns all possible
// solutions (up to 4). Follows steps from the paper "A Novel Parameterization
// of the Perspective-Three-Point Problem for a direct computation of Absolute
// Camera position and Orientation" by Kneip et. al.
//
// Params:
//   image_points: Location of features on the image plane (x[i][*] = i-th image
//     point)
//   world_points: 3D location of features. Must correspond to the image_point
//     of the same index (x[i][*] = i-th world point)
//   focal_length: x, then y focal length (expressed in pixels).
//   principle point: x, then y image optical center point (in pixels).
//   rotation: The candidate rotations computed from the 3 point algorithm.
//   translation: The candidate translations computed.
// NOTE: P3P returns up to 4 poses, so the rotation and translation arrays are
//       indeed arrays of 3x3 and 3x1 arrays respectively.
// Return: the number of poses computed.
int PoseThreePoints(const double image_points[3][2],
                    const double world_points[3][3],
                    const double focal_length[2],
                    const double principle_point[2],
                    double rotation[][3][3],
                    double translation[][3]);

// Computes pose using three point algorithm (method above). The fourth
// correspondence is used to determine the best solution of the (up to 4)
// candidate solutions. Same parameters as above, except only the best solution
// is returned in the output parameters, rotation and translation.
// Return: true if a successful pose is found, false else.
bool PoseFourPoints(const double image_points[4][2],
                    const double world_points[4][3],
                    const double focal_length[2],
                    const double principle_point[2],
                    double rotation[3][3],
                    double translation[3]);
}  // pose
}  // vision

#endif  // VISION_POSE_PERSPECTIVE_THREE_POINT_H_
