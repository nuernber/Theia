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

#ifndef THEIA_VISION_TRANSFORMATION_ALIGN_POINT_CLOUDS_H_
#define THEIA_VISION_TRANSFORMATION_ALIGN_POINT_CLOUDS_H_

namespace theia {
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
void AlignPointCloudsICP(const int num_points,
                         const double left[],
                         const double right[],
                         double rotation[3 * 3],
                         double translation[3]);

// Computes the orientation, position, and scale factor for the transformation
// between two corresponding 3D point sets A and B such as they are related by:
//
//     B = s * R * A + t
//
// where A is "left" and B is "right". Implementation is based on the paper by
// Umeyama "Least-squares estimation of transformation parameters between two
// point patterns".
void AlignPointCloudsUmeyama(const int num_points,
                             const double left[],
                             const double right[],
                             double rotation[3 * 3],
                             double translation[3],
                             double* scale);

}  // namespace theia

#endif  // THEIA_VISION_TRANSFORMATION_ALIGN_POINT_CLOUDS_H_
