#include "vision/pose/perspective_three_point.h"

#include <iostream>
#include <math.h>
#include "math/closed_form_polynomial_solver.h"
#include <algorithm>

namespace vision {
namespace pose {

namespace {

double kEpsilon = 1e-4;

// Using the way in the overview website.
void GetQuarticCoefficients(double a,
                            double b,
                            double cos_uv,
                            double cos_uw,
                            double cos_vw,
                            double coeff[5]) {
  double p = 2.0*cos_vw;
  double q = 2.0*cos_uw;
  double r = 2.0*cos_uv;
  coeff[4] = a*a + b*b - 2.0*a - 2.0*b + 2.0*(1.0 - r*r)*b*a + 1;
  coeff[3] = -2.0*q*a*a - r*p*b*b + 4.0*q*a + (2.0*q + p*r)*b +
      (r*r*q - 2.0*q + r*p)*a*b - 2.0*q;
  coeff[2] = (2.0 + q*q)*a*a + (p*p + r*r - 2.0)*b*b - (4.0 + 2.0*q*q)*a -
      (p*q*r + p*p)*b - (p*q*r + r*r)*a*b + q*q + 2;
  coeff[1] = -2.0*q*a*a - r*p*b*b + 4.0*q*a + (p*r + q*p*p - 2.0*q)*b +
      (r*p + 2.0*q)*a*b - 2.0*q;
  coeff[0] = a*a + b*b - 2.0*a + (2.0 - p*p)*b -2.0*a*b + 1.0;
}

// Using Fischler's way.
void GetQuarticCoefficients2(double k1,
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

  if(m_prime*q == m*q_prime) {
    double sqrt_temp = sqrt(cos_ac*cos_ac + (ac*ac - a*a)/(a*a));
    y[0] = cos_ac + sqrt_temp;
    y[1] = cos_ac - sqrt_temp;
    return 2;
  }

  y[0] = (p_prime*q - p*q_prime)/(m*q_prime - m_prime*q);
  return 1;
}

void SolveForYGivenX(double a,
                     double b,
                     double cos_uv,
                     double cos_uw,
                     double cos_vw,
                     const double x[],
                     int num_solutions,
                     double y[]) {
  double p = 2.0*cos_vw;
  double q = 2.0*cos_uw;
  double r = 2.0*cos_uv;

  // The following are an impossibly ugly set of equations. See
  // http://iplimage.com/blog/p3p-perspective-point-overview/ for the proof.
  double b_temp = (p*p - p*q*r + r*r)*a + (p*p - r*r)*b - p*p + p*q*r - r*r;
  double b1 = b*b_temp*b_temp;

  // Precompute a few of the coeffs for readability if nothing else.
  // Multiply by x^3.
  double coeff3 = r*r*r*(a*a + b*b - 2.0*a - 2.0*b + (2.0 - r*r)*a*b + 1.0);
  // Multiply by x^2.
  double coeff2 = r*r*(p + p*a*a -2.0*r*q*a*b + 2.0*r*q*b - 2.0*r*q - 2.0*p*a -
                       2.0*p*b + p*r*r*b + 4.0*r*q*a + q*r*r*r*a*b -
                       2.0*r*q*a*a + 2.0*p*a*b + p*b*b - r*r*p*b*b);
  // Multiply by x.
  double coeff1 =
      r*r*r*r*r*(b*b - a*b) - r*r*r*r*p*q*b +
      r*r*r*(q*q - 4.0*a - 2.0*q*q*a + q*q*a*a + 2.0*a*a - 2.0*b*b + 2.0) +
      r*r*(4.0*p*q*a - 2.0*p*q*a*b + 2.0*p*q*b - 2.0*p*q - 2.0*p*q*a*a) +
      r*(p*p*b*b - 2.0*p*p*b + 2.0*p*p*a*b - 2.0*p*p*a + p*p + p*p*a*a);
  // Constant term, no need to multiply by any degree of x.
  double coeff0 =
      (2.0*p*r*r - 2.0*r*r*r*q + p*p*p - 2.0*p*p*q*r + p*q*q*r*r)*a*a +
      (p*p*p - 2.0*p*r*r)*b*b +
      (4.0*q*r*r*r - 4.0*p*r*r - 2.0*p*p*p + 4.0*p*p*q*r - 2.0*p*q*q*r*r)*a +
      (-2.0*q*r*r*r + p*r*r*r*r + 2.0*p*p*q*r - 2.0*p*p*p)*b +
      (2.0*p*p*p + 2*q*r*r*r - 2.0*p*p*q*r)*a*b +
      p*q*q*r*r - 2.0*p*p*q*r + 2.0*p*r*r + p*p*p - 2.0*r*r*r*q;

  // Solve for y plugging in x-values. b_1*y = b_0.
  for (int i = 0; i < num_solutions; i++) {
    double temp = (1.0 - a - b)*x[i]*x[i] + (a - 1.0)*q*x[i] - a + b + 1.0;
    double b0 = temp*(coeff3*x[i]*x[i]*x[i] +
                      coeff2*x[i]*x[i] +
                      coeff1*x[i] +
                      coeff0);
    y[i] = b0/b1;
  }
}

// Solve for the 3x3 rotation matrix using the method proposed by Horn (1986)
// for using quaternions to solve a closed form quartic. The solution returns
// the rotation transformation of the right points to the left points. This is
// under that assumption that the translation of points has already been solved
// (i.e. the origin of both point sets is set to the centroid of that
// set). Returns a 3x3 rotation matrix.
void SolveRotationLeastSquares(const double left_points[][3],
                               const double right_points[][3],
                               int num_points,
                               double rotation[3][3]) {
  // Solve rotation using Horn's closed form method on quaternions. See paper
  // for notation.
  // Let s_ij = s_xx, s_xy, etc. where x = 0, y = 1, z = 2 in the loop
  double s[3][3];

  // for each point.
  for (int i = 0; i < num_points; i++)
    // for each dimension of the left points
    for (int j = 0; j < 3; j++)
      // for each dimension of the right points
      for (int k = 0; k < 3; k++)
        s[j][k] += left_points[i][j]*right_points[i][k];

  // Compute the real, symmetric matrix N.
  double a = s[0][0] + s[1][1] + s[2][2];
  double b = s[0][0] - s[1][1] - s[2][2];
  double c = -1.0*s[0][0] + s[1][1] - s[2][2];
  double d = -1.0*s[0][0] - s[1][1] + s[2][2];
  double e = s[1][2] - s[2][1];
  double f = s[0][1] + s[1][0];
  double g = s[1][2] + s[2][1];
  double h = s[2][0] - s[0][2];
  double i = s[2][0] + s[0][2];
  double j = s[0][1] - s[1][0];

  // Create the coefficients for the closed form quartic to solve for the
  // eigenvalues.
  // c3 = a + b + c + d = trace of N = 0.
  double c3 = 0.0;
  double c2 = -2.0*(s[0][0]*s[0][0] + s[0][1]*s[0][1] + s[0][2]*s[0][2] +
                    s[1][2]*s[1][2] + s[1][1]*s[1][1] + s[1][2]*s[1][2] +
                    s[2][0]*s[2][0] + s[2][1]*s[2][1] + s[2][2]*s[2][2]);
  double c1 =
      8.0*(s[0][0]*s[1][2]*s[2][1] + s[1][1]*s[2][0]*s[0][2] +
           s[2][2]*s[0][1]*s[1][0]) -
      8.0*(s[0][0]*s[1][1]*s[2][2] + s[1][2]*s[2][0]*s[0][1] +
           s[2][1]*s[1][0]*s[0][2]);
  double c0 = (a*b - e*e)*(c*d - g*g) + (e*h - a*f)*(f*d - g*i) +
      (a*i - e*j)*(f*g - c*i) + (e*f - b*h)*(h*d - g*j) +
      (b*j - e*i)*(h*g - c*j) + (h*i - f*j)*(h*i - f*j);

  // Solve for the eigenvalues of N.
  double eigenvalues[4];
  int num_eigenvalues =
      math::SolveQuarticReals(1.0, c3, c2, c1, c0, eigenvalues);

  // Get the largest positive eigenvalue.
  double max_eigenvalue = 0.0;
  for (int i = 0; i < num_eigenvalues; i++)
    max_eigenvalue =
        eigenvalues[i] > max_eigenvalue ? eigenvalues[i] : max_eigenvalue;

  // Plug in the eigenvalue to find the corresponding eigenvector. This
  // eigenvector represents the quaternion for optimal rotation.
  double q[4];
  q[0] = a/max_eigenvalue;
  q[1] = b/max_eigenvalue;
  q[2] = c/max_eigenvalue;
  q[3] = d/max_eigenvalue;

  // Calculate the square of the normalization factor. The final rotation matrix
  // is only ever divided by the square, so it is a waste to calculate the sqrt.
  double norm_sq = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];

  // Convert quaternion to Rotation matrix.
  rotation[0][0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
  rotation[0][1] = 2.0*(q[1]*q[2] - q[0]*q[3]);
  rotation[0][2] = 2.0*(q[1]*q[3] + q[0]*q[2]);
  rotation[1][0] = 2.0*(q[1]*q[2] + q[0]*q[3]);
  rotation[1][1] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
  rotation[1][2] = 2.0*(q[2]*q[3] - q[0]*q[1]);
  rotation[2][0] = 2.0*(q[1]*q[3] - q[0]*q[2]);
  rotation[2][1] = 2.0*(q[2]*q[3] + q[0]*q[1]);
  rotation[2][2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      rotation[i][j] /= norm_sq;
}

void GetNearestNeighbor(const double model[][3],
                        int num_models,
                        const double point[3],
                        double nearest_neighbor[3]) {
  double sq_dist = 1e9;
  int nn_index = 0;
  for (int i = 0; i < num_models; i++) {
    double temp_sq_dist = (point[0] - model[i][0])*(point[0] - model[i][0]) +
        (point[1] - model[i][1])*(point[1] - model[i][1]) +
        (point[2] - model[i][2])*(point[2] - model[i][2]);
    if (temp_sq_dist < sq_dist) {
      sq_dist = temp_sq_dist;
      nn_index = i;
    }
  }
  for (int i = 0; i < 3; i++)
    nearest_neighbor[i] = model[nn_index][i];
}

// Use Besel-McKay registration with ICP to align point clouds. Could also use
// SVD to determine the rotation by calculating the SVD on a cross-covariance
// based matrix, but that is excessive computation for a 3x3
// matrix. Additionally, we use quaternions, then at the end transform it back
// into a 3x3 rotation matrix using the closed-form solution proposed by Horn
// (1986). Our goal is to align the points to the model.
void BeselMcKayICP(const double model[3][3],
                   const double points[3][3],
                   double rotation[3][3],
                   double translation[3]) {
  // Compute and align centroids.

  double model_centroid[3] = {0.0, 0.0, 0.0};
  double points_centroid[3] = {0.0, 0.0, 0.0};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      model_centroid[j] += model[i][j]/3.0;
      points_centroid[j] += points[i][j]/3.0;
    }
  }

  // Translation = alignment of centroids.
  for (int i = 0; i < 3; i++)
    translation[i] = model_centroid[i] - points_centroid[i];

  // Shift points to be centered around the centroid.
  double model_prime[3][3];
  double point_prime[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      model_prime[i][j] = model[i][j] - model_centroid[j];
      point_prime[i][j] = points[i][j] - points_centroid[j];
    }
  }

  double least_squares_error = 0;
  double least_squares_diff = 1e9;
  // Exit this loop when the difference in consecutive iterations is small
  // enough. Rotation will be set the the rotation that produced this smallest
  // error.
  while (least_squares_diff > kEpsilon) {
    // Find NN of each point.
    double closest_model_points[3][3];
    for (int i = 0; i < 3; i++)
      GetNearestNeighbor(model_prime, 3, point_prime[i], closest_model_points[i]);

    // Solve for rigid transformation that satisfies the least square fit.
    SolveRotationLeastSquares(closest_model_points,
                              point_prime,
                              3,
                              rotation);

    // Apply rotation and calculate the squared distance.
    double candidate_squared_distance = 0.0;
    // i = dimension (e.g. x, y, or z)
    for (int i = 0; i < 3; i++) {
      // j = point to apply rotation to.
      for (int j = 0; j < 3; j++) {
        double candidate_point = rotation[i][0]*point_prime[j][0] +
            rotation[i][1]*point_prime[j][1] +
            rotation[i][2]*point_prime[j][2];
        candidate_squared_distance =
            (point_prime[j][i] - candidate_point)*
            (point_prime[j][i] - candidate_point);
      }
    }

    least_squares_diff = least_squares_error - candidate_squared_distance;
    least_squares_error = candidate_squared_distance;
  }
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

  std::cout << "cos_uv = " << cos_uv << " cos_uw = " << cos_uw
            << " cos_vw = " << cos_vw << std::endl;

  double a = (bc*bc)/(ab*ab);
  double b = (ac*ac)/(ab*ab);
  double a_coeff[5];

  GetQuarticCoefficients(a, b, cos_uv, cos_uw, cos_vw, a_coeff);
  for (int i = 0; i < 5; i++)
    std::cout << "quartic coeffs: " << a_coeff[i] << std::endl;

  double x[4];
  int num_solutions = math::SolveQuarticReals(a_coeff[4],
                                              a_coeff[3],
                                              a_coeff[2],
                                              a_coeff[1],
                                              a_coeff[0],
                                              x);
  std::cout << "Num solutions to quartic: " << num_solutions << std::endl;
  double y[4];
  SolveForYGivenX(a, b, cos_uv, cos_uw, cos_vw, x, num_solutions, y);

  for (int i = 0; i < num_solutions; i++)
    std::cout << "(x, y) = (" << x[i] << ", " << y[i] << ")" << std::endl;
  double pa[num_solutions];
  double pb[num_solutions];
  double pc[num_solutions];
  for (int i = 0; i < num_solutions; i++) {
    pc[i] = (ab*ab)/(x[i]*x[i] + y[i]*y[i] - x[i]*y[i]*cos_uv);
    pb[i] = y[i]*pc[i];
    pa[i] = x[i]*pc[i];

    // Note: the A, B, C referenced in step 4 are not the same A, B, C at the
    // beginning (referencing world coordinates). They are actually the 3D
    // coordinates from the optical center coordinate system, so we will denote
    // them as world_points_prime.
    double world_points_prime[3][3];
    for (int j = 0; j < 3; j++) {
      world_points_prime[0][j] = u[j]*pa[i];
      world_points_prime[1][j] = v[j]*pb[i];
      world_points_prime[2][j] = w[j]*pc[i];
    }

    // Align world_points and world_points_prime with Besel-McKay
    // registration. This will give us the pose.
    BeselMcKayICP(world_points, world_points_prime, rotation[i], translation[i]);
  }

  return num_solutions;
}
// Computes camera pose using the three point algorithm and returns all possible
// solutions (up to 4). u = index 0, v = index 1, w = index 2 of image/world
// points.
int PoseThreePoints2(const double image_points[3][2],
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

  ab = bc = ac = 2*sqrt(3);
  cos_uv = cos_uw = cos_vw = 20.0/32.0;
  
  std::cout << "cos_uv = " << cos_uv << " cos_uw = " << cos_uw
            << " cos_vw = " << cos_vw << std::endl;

  double k1 = (bc*bc)/(ab*ab);
  double k2 = (ac*ac)/(ab*ab);
  double a_coeff[5];

  // Fischler way.
  GetQuarticCoefficients2(k1, k1, cos_uv, cos_uw, cos_vw, a_coeff);

  // Rig it for now.
  a_coeff[0] = -0.5625;
  a_coeff[1] = 3.515625;
  a_coeff[2] = -5.90625;
  a_coeff[3] = 3.515625;
  a_coeff[4] = -0.5625;
  ab = bc = ac = 2*sqrt(3);
  cos_uv = cos_uw = cos_vw = 20.0/32.0;
  
  
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
        BeselMcKayICP(world_points,
                      world_points_prime,
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
  for (int i = 0; i < num_valid_poses; i++) {

  }
  return true;
}
}  // pose
}  // vision
