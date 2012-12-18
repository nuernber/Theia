#include "gtest/gtest.h"
#include "vision/pose/perspective_three_point.h"
#include <math.h>
#include <Eigen/Core>

using namespace Eigen;

namespace vision {
namespace pose {
namespace {
double kEpsilon = 1e-9;

// Returns a random double between dMin and dMax
double RandDouble(double dMin, double dMax) {
  double d = static_cast<double>(rand()) / RAND_MAX;
  return dMin + d * (dMax - dMin);
}
}

TEST(PerspectiveThreePoint, Sanity) {
  // World coordinates of the 4 control points. Let them be random points in the
  // 2x2x2 centered around the origin.
  double world_points[4][3];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++)
      world_points[i][j] = RandDouble(-2, 2);

  // Find the camera projections of each of these points.
  // Make the camera at (0, 0, 6) looking straight down the axis.
  Matrix<double, 3, 4> transformation;
  transformation << 1, 0, 0, 0,
      0, -1, 0, 0,
      0, 0, -1, 8;
  
  //Intrinsic parameters of camera
  double focal_length[] = {800, 800};
  double principle_point[] = {320, 240};

  Matrix3d camera_matrix;
  camera_matrix << focal_length[0], 0, principle_point[0],
      0, focal_length[1], principle_point[1],
      0, 0, 1.0;

  double image_points[4][2];
  for (int i = 0; i < 4; i++) {
    Vector4d world_pt(world_points[i][0],
                      world_points[i][1],
                      world_points[i][2],
                      1.0);
    Vector3d proj_point = camera_matrix*transformation*world_pt;
    image_points[i][0] = proj_point[0]/proj_point[2];
    image_points[i][1] = proj_point[1]/proj_point[2];
  }
    
  double rotation[4][3][3];
  double translation[4][3];
  clock_t t = clock();
  int num_solutions = PoseThreePoints(image_points,
                                      world_points,
                                      focal_length,
                                      principle_point,
                                      rotation,
                                      translation);
  ASSERT_GT(num_solutions, 0);
  
  double best_rotation[3][3];
  double best_translation[3];
  bool solved = PoseFourPoints(image_points,
                               world_points,
                               focal_length,
                               principle_point,
                               best_rotation,
                               best_translation);
  
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      ASSERT_LT(std::abs(best_rotation[i][j] - transformation(i,j)), kEpsilon);
    }
    ASSERT_LT(std::abs(best_translation[i] - transformation(i,3)), kEpsilon);
  }
}
}  // namespace pose
}  // namespace vision
