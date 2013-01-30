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

#ifndef VISION_TRANSFORMATION_ALIGN_POINT_CLOUDS_H_
#define VISION_TRANSFORMATION_ALIGN_POINT_CLOUDS_H_

namespace vision {
namespace transformation {
// Use Besl-McKay registration to align point clouds. We use SVD decomposition
// to find the rotation, as this is much more likely to find the global minimum
// as compared to traditional ICP, which is only guaranteed to find a local
// minimum. Our goal is to find the transformation from the left to the right
// coordinate system. We assume that the left and right models have the same
// number of points, and that the points are aligned by correspondence
// (i.e. left[i] corresponds to right[i]).
//
// NOTE: SVD is indeed excessive for a 3 dimensional problem, as it leads to SVD
// decomposition of a 3x3. However, after running multiple tests with
// traditional ICP, it was noted that the local minimum achieved was often not
// satisfactory. If runtime becomes an issue, then I may consider switching back
// to an (improved) true ICP method, as it is an analytic and closed form
// solution so it is very fast. Because of this, this is not a true
// ICP method, but it achieves the same registration using a method suggested by
// Besl-McKay.
void AlignPointClouds(const double left[][3],
                      const double right[][3],
                      int num_points,
                      double rotation[3][3],
                      double translation[3]);
}  // namespace transformation
}  // namespace vision
#endif  // VISION_TRANSFORMATION_ALIGN_POINT_CLOUDS_H_
