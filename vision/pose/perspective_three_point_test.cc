#include "gtest/gtest.h"
#include "vision/pose/perspective_three_point.h"
#include <time.h>
namespace vision {
namespace pose {

TEST(PerspectiveThreePoint, test1) {
  //World coordinates of the 3 control points
  double world_points[3][3] = {{0.105, 0.07245, 0},
                               {-0.105, 0.07425, 0},
                               {-0.105, -0.07425, 0}};

  //Image coordinates of the 3 points
  double image_points[3][2] = {{256, 218},
                               {591, 224},
                               {588, 461}};

  //Intrinsic parameters of camera
  double focal_length[] = {990.08190, 991.42390};
  double principle_point[] = {402.16577, 291.72516};

  double rotation[4][3][3];
  double translation[4][3];
  clock_t t = clock();
  int num_solutions = PoseThreePoints(image_points,
                                      world_points,
                                      focal_length,
                                      principle_point,
                                      rotation,
                                      translation);
  t = clock() - t;
  printf ("It took me %d clicks (%f seconds).\n",t,((float)t)/CLOCKS_PER_SEC);
  //Output all the probable extrinsic parameters
  for(int i = 0; i < num_solutions; i++)
  {
    std::cout << "Solution: " << i << std::endl;
    for (int j = 0; j < 3; j++) {
      std::cout << rotation[i][j][0] << " " << rotation[i][j][1] << " "
                << rotation[i][j][2] << " | " << translation[i][j] << std::endl;
    }
  }
}

}  // namespace pose
}  // namespace vision
