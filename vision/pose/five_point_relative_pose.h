#ifndef VISION_POSE_FIVE_POINT_RELATIVE_POSE_H_
#define VISION_POSE_FIVE_POINT_RELATIVE_POSE_H_

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
//   essential_matrix: Output candidate solutions of the 5 point algorithm.
// Return: the number of poses computed.
int FivePointRelativePose(const double image1_points[3][2],
                          const double image2_points[3][2],
                          double essential_matrix[][3][3]);
}  // pose
}  // vision

#endif  // VISION_POSE_FIVE_POINT_RELATIVE_POSE_H_
