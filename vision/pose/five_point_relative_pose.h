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
