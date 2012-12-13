#ifndef VISION_POSE_PERSPECTIVE_THREE_POINT_H_
#define VISION_POSE_PERSPECTIVE_THREE_POINT_H_

namespace vision {
namespace pose {
// Computes camera pose using the three point algorithm and returns all possible
// solutions (up to 4). Follows steps from the overview at
// http://iplimage.com/blog/p3p-perspective-point-overview/
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

int PoseThreePoints2(const double image_points[3][2],
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
