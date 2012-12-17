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
