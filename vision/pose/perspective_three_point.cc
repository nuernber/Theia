#include "vision/pose/perspective_three_point.h"

#include <iostream>
#include <math.h>
#include "math/closed_form_polynomial_solver.h"
#include "vision/transformation/align_point_clouds.h"
#include <algorithm>

namespace vision {
namespace pose {

namespace {

double kEpsilon = 1e-4;

// Using Fischler's way.
void GetQuarticCoefficients(double k1,
                            double k2,
                            double cos_ab,
                            double cos_ac,
                            double cos_bc,
                            double coeff[5]) {
  coeff[4] = (k1*k2 - k1 - k2)*(k1*k2 - k1 - k1) - 4.0*k1*k2*cos_bc*cos_bc;
  coeff[3] = 4.0*(k1*k2 - k1 - k2)*k2*(1 - k1)*cos_ab +
      4.0*k1*cos_bc*((k1*k2 + k2 -k1)*cos_ac + 2*k2*cos_ab*cos_bc);
  coeff[2] = (2.0*k2*(1.0 - k1)*cos_ab)*(2.0*k2*(1.0 - k1)*cos_ab) +
      2.0*(k1*k2 + k1 - k2)*(k1*k2 - k1 - k2) +
      4.0*k1*((k1 - k2)*(cos_bc*cos_bc) + (1 - k2)*k1*cos_ac*cos_ac
              - 2.0*k2*(1 + k1)*cos_ab*cos_ac*cos_bc);
  coeff[1] = 4.0*(k1*k2 + k1 - k2)*k2*(1.0 - k1)*cos_ab +
      4.0*k1*((k1*k2 - k1 + k2)*cos_ac*cos_bc + 2.0*k1*k2*cos_ab*cos_ac*cos_ac);
  coeff[0] = (k1*k2 + k1 - k2)*(k1*k2 - k1 - k2) - 4.0*k1*k1*k2*cos_ac*cos_ac;
}

// Project a 3D point into the image plan with the equation:
// projected_point = [camera_matrix][R|t][world_point] where camera_matrix
// world_point is a 3 dimensional vector in world coordinates.
void ProjectPoint(const double camera_matrix[3][3],
                  const double rotation[3][3],
                  const double translation[3],
                  const double world_point[3],
                  double projected_point[2]) {
  // Create the 3x4 camera matrix. This makes multiplication later easy.
  double projection_matrix[3][4];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      projection_matrix[i][j] = camera_matrix[i][0]*rotation[0][j] +
          camera_matrix[i][1]*rotation[1][j] +
          camera_matrix[i][2]*rotation[2][j];
    }
    projection_matrix[i][3] = camera_matrix[i][0]*translation[0] +
        camera_matrix[i][1]*translation[1] + camera_matrix[i][2]*translation[2];
  }

  double homogeneous_point[3];
  for (int i = 0; i < 3; i++) {
    homogeneous_point[i] = projection_matrix[i][0]*world_point[0] +
        projection_matrix[i][1]*world_point[1] +
        projection_matrix[i][2]*world_point[2] +
        projection_matrix[i][3];
  }
  projected_point[0] = homogeneous_point[0]/homogeneous_point[2];
  projected_point[1] = homogeneous_point[1]/homogeneous_point[2];
}

int PruneArray(double input[],
               int num_elements) {
  std::sort(&input[0], &input[num_elements]);
  return std::unique(&input[0], &input[num_elements]) - &input[0];
}

int SolveFischlerY(double x,
                   double k1,
                   double k2,
                   double a,
                   double ac,
                   double cos_ab,
                   double cos_ac,
                   double cos_bc,
                   double y[]) {
  double m = 1.0 - k1;
  double p = 2.0*(k1*cos_ac - x*cos_bc);
  double q = x*x - k1;

  double m_prime = 1.0;
  double p_prime = 2*(-x*cos_bc);
  double q_prime = x*x*(1 - k2) + 2.0*x*k2*cos_ab - k2;

  if (m_prime*q == m*q_prime) {
    double sqrt_temp = sqrt(cos_ac*cos_ac + (ac*ac - a*a)/(a*a));
    y[0] = cos_ac + sqrt_temp;
    y[1] = cos_ac - sqrt_temp;
    return 2;
  }

  y[0] = (p_prime*q - p*q_prime)/(m*q_prime - m_prime*q);
  return 1;
}
}  // namespace

// Computes camera pose using the three point algorithm and returns all possible
// solutions (up to 4). u = index 0, v = index 1, w = index 2 of image/world
// points.
int PoseThreePoints(const double image_points[3][2],
                    const double world_points[3][3],
                    const double focal_length[2],
                    const double principle_point[2],
                    double rotation[][3][3],
                    double translation[][3]) {
  // Normalize image points and project them onto a unit sphere centered at P,
  // the center of projection.
  double normalized_image_points[3][3];
  for (int i = 0; i < 3; i++) {
    std::cout << image_points[i][0] << " " << image_points[i][1] << std::endl;
    // Normalize x.
    double temp_x =
        (image_points[i][0] - principle_point[0])/(focal_length[0]);
    // Normalize y.
    double temp_y =
        (image_points[i][1] - principle_point[1])/(focal_length[1]);
    // Skip normalizing z since z = 1.0.

    // Then normalize the whole thing using L2 norm.
    double norm = sqrt(temp_x*temp_x + temp_y*temp_y + 1.0);
    normalized_image_points[i][0] = temp_x/norm;
    normalized_image_points[i][1] = temp_y/norm;
    normalized_image_points[i][2] = 1.0/norm;
  }

  // Define shorthand macros for readability.
#define u normalized_image_points[0]
#define v normalized_image_points[1]
#define w normalized_image_points[2]

  for (int i = 0; i < 3; i++)
    std::cout << u[i] << " " << v[i] << " " << w[i] << std::endl;

  double cos_uv = u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
  double cos_uw = u[0]*w[0] + u[1]*w[1] + u[2]*w[2];
  double cos_vw = v[0]*w[0] + v[1]*w[1] + v[2]*w[2];

  double ab = sqrt((v[0] - u[0])*(v[0] - u[0]) +
                   (v[1] - u[1])*(v[1] - u[1]) +
                   (v[2] - u[2])*(v[2] - u[2]));
  double bc = sqrt((w[0] - v[0])*(w[0] - v[0]) +
                   (w[1] - v[1])*(w[1] - v[1]) +
                   (w[2] - v[2])*(w[2] - v[2]));
  double ac = sqrt((w[0] - u[0])*(w[0] - u[0]) +
                   (w[1] - u[1])*(w[1] - u[1]) +
                   (w[2] - u[2])*(w[2] - u[2]));

  //  ab = bc = ac = 2*sqrt(3);
  // cos_uv = cos_uw = cos_vw = 20.0/32.0;

  std::cout << "cos_uv = " << cos_uv << " cos_uw = " << cos_uw
            << " cos_vw = " << cos_vw << std::endl;

  double k1 = (bc*bc)/(ab*ab);
  double k2 = (ac*ac)/(ab*ab);
  double a_coeff[5];

  // Fischler way.
  GetQuarticCoefficients(k1, k1, cos_uv, cos_uw, cos_vw, a_coeff);

  // Rig it for now.
  /*
    a_coeff[0] = -0.5625;
    a_coeff[1] = 3.515625;
    a_coeff[2] = -5.90625;
    a_coeff[3] = 3.515625;
    a_coeff[4] = -0.5625;
    ab = bc = ac = 2*sqrt(3);
    cos_uv = cos_uw = cos_vw = 20.0/32.0;
  */

  for (int i = 0; i < 5; i++)
    std::cout << "quartic coeffs2: " << a_coeff[i] << std::endl;

  double x[4];
  int num_x_solutions = math::SolveQuarticReals(a_coeff[4],
                                                a_coeff[3],
                                                a_coeff[2],
                                                a_coeff[1],
                                                a_coeff[0],
                                                x);
  num_x_solutions = PruneArray(x, num_x_solutions);
  std::cout << "Num solutions to quartic: " << num_x_solutions << std::endl;
  int num_pose_solutions = 0;
  for (int i = 0; i < num_x_solutions; i++) {
    std::cout << "Evaluating x = " << x[i] << std::endl;
    // Only accept positive, real roots.
    if (x[i] <= 0.0)
      continue;

    double a = ab/(sqrt(x[i]*x[i] - 2*x[i]*cos_uv + 1.0));
    double b = a*x[i];
    double y[2];
    int num_y_solutions =
        SolveFischlerY(x[i], k1, k2, a, ac, cos_uv, cos_uw, cos_vw, y);

    for (int j = 0; j < num_y_solutions; j++) {
      double c = y[j]*a;
      double dist_check = sqrt(b*b + c*c - 2.0*b*c*cos_vw);
      // Make sure the y candidate solution passes the check in Eq 3. of
      // Fischler's derivation.
      if (abs(dist_check - bc) < kEpsilon) {
        num_pose_solutions++;
        std::cout << "x = " << x[i] << " y = " << y[j] << std::endl;
        std::cout << "a = " << a << " b = " << b << " c = " << c << std::endl;

        // Note: the A, B, C referenced in step 4 are not the same A, B, C at
        // the beginning (referencing world coordinates). They are actually the
        // 3D coordinates from the optical center coordinate system, so we will
        // denote them as world_points_prime.
        double world_points_prime[3][3];
        for (int j = 0; j < 3; j++) {
          world_points_prime[0][j] = u[j]*a;
          world_points_prime[1][j] = v[j]*b;
          world_points_prime[2][j] = w[j]*c;
        }

        // Align world_points and world_points_prime with Besel-McKay
        // registration. This will give us the pose.
        vision::transformation::AlignPointClouds(world_points,
                                                 world_points_prime,
                                                 3,
                                                 rotation[i],
                                                 translation[i]);
      }
    }
  }

  return num_pose_solutions;
}

// Computes pose using three point algorithm. The fourth correspondence is used
// to determine the best solution of the (up to 4) candidate solutions.
bool PoseFourPoints(const double image_points[4][2],
                    const double world_points[4][3],
                    const double focal_length[2],
                    const double principle_point[2],
                    double rotation[3][3],
                    double translation[3]) {
  double first_three_image_points[3][2];
  double first_three_world_points[3][3];
  for (int i = 0; i < 3; i++) {
    first_three_image_points[i][0] = image_points[i][0];
    first_three_image_points[i][1] = image_points[i][1];
    first_three_image_points[i][2] = image_points[i][2];

    first_three_world_points[i][0] = world_points[i][0];
    first_three_world_points[i][1] = world_points[i][1];
  }

  double candidate_rotation[4][3][3];
  double candidate_translation[4][3];
  int num_valid_poses = PoseThreePoints(first_three_image_points,
                                        first_three_world_points,
                                        focal_length,
                                        principle_point,
                                        candidate_rotation,
                                        candidate_translation);
  // For each candidate pose, measure the reprojection error of the 4th point
  // and pick the best candidate pose.
  double camera_matrix[3][3] = {{focal_length[0], 0, principle_point[0]},
                                {0, focal_length[1], principle_point[1]},
                                {0, 0, 1.0}};
  double min_reprojection_error = 0;
  int best_pose_index = 0;
  for (int i = 0; i < num_valid_poses; i++) {
    double projected_point[2];
    ProjectPoint(camera_matrix,
                 candidate_rotation[i],
                 candidate_translation[i],
                 world_points[3],
                 projected_point);
    double reprojection_error =
        (projected_point[0] - image_points[3][0])*
        (projected_point[0] - image_points[3][0]) +
        (projected_point[1] - image_points[3][1])*
        (projected_point[1] - image_points[3][1]);
    if (reprojection_error < min_reprojection_error) {
      min_reprojection_error = reprojection_error;
      best_pose_index = i;
    }
  }
  rotation = candidate_rotation[best_pose_index];
  translation = candidate_translation[best_pose_index];
  return true;
}
}  // pose
}  // vision
