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

#ifndef THEIA_VISION_SFM_TRIANGULATION_TRIANGULATION_H_
#define THEIA_VISION_SFM_TRIANGULATION_TRIANGULATION_H_

#include <Eigen/Core>
#include <vector>

#include "theia/vision/sfm/projection_matrix.h"

namespace theia {
// Triangulates 2 posed views using the DLT method from HZZ 12.2 p 312. The
// inputs are the projection matrices and the unit image observations.
Eigen::Vector4d Triangulate(const TransformationMatrix& pose_left,
                            const TransformationMatrix& pose_right,
                            const Eigen::Vector3d& point_left,
                            const Eigen::Vector3d& point_right);

// Computes n-view triangulation by computing the SVD that wil approximately
// minimize reprojection error. The inputs are the projection matrices and the
// unit image observations.
Eigen::Vector4d TriangulateNViewSVD(
    const std::vector<TransformationMatrix>& poses,
    const std::vector<Eigen::Vector3d>& points);

// Computes n-view triangulation by an efficient L2 minimization of the
// algebraic error. This minimization is independent of the number of points, so
// it is extremely scalable. It gives better reprojection errors in the results
// and is significantly faster. The inputs are the projection matrices and the
// unit image observations.
Eigen::Vector4d TriangulateNView(const std::vector<TransformationMatrix>& poses,
                                 const std::vector<Eigen::Vector3d>& points);

}  // namespace theia

#endif  // THEIA_VISION_SFM_TRIANGULATION_TRIANGULATION_H_
