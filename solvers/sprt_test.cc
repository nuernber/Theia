#include <math.h>
#include <vector>

#include "gtest/gtest.h"
#include "solvers/estimator.h"
#include "solvers/sequential_probability_ratio_test.h"

using std::vector;

namespace solvers {
namespace {
struct Point {
  double x;
  double y;
  Point() {}
  Point(double _x, double _y) : x(_x), y(_y) {}
};

// y = mx + b
struct Line {
  double m;
  double b;
  Line() {}
  Line(double _m, double _b) : m(_m), b(_b) {}
};

class LineEstimator : public Estimator<Point, Line> {
 public:
  LineEstimator() {}
  ~LineEstimator() {}

  bool EstimateModel(const vector<Point>& data, Line* model) const {
    model->m = (data[1].y - data[0].y)/(data[1].x - data[0].x);
    model->b = data[1].y - model->m*data[1].x;
    return true;
  }

  double Error(const Point& point, const Line& line) const {
    double a = -1.0*line.m;
    double b = 1.0;
    double c = -1.0*line.b;

    return fabs(a*point.x + b*point.y + c)/(sqrt(pow(a*a + b*b, 2)));
  }
};

// Returns a random double between dMin and dMax
double RandDouble(double dMin, double dMax) {
  double d = static_cast<double>(rand()) / RAND_MAX;
  return dMin + d * (dMax - dMin);
}
}  // namespace

// TODO(cmsweeney): Make this test a verification (i.e. is the value coming out
// accurate?) instead of just a sanity check.
TEST(SPRTTest, CalculateSPRTDecisionThreshold) {
  double sigma = 0.05;
  double epsilon = 0.1;
  double decision_threshold = CalculateSPRTDecisionThreshold(sigma, epsilon);
  std::cout << "Decision threshold: " << decision_threshold << std::endl;

  // Test with change of values for timing.
  decision_threshold = CalculateSPRTDecisionThreshold(sigma,
                                                      epsilon,
                                                      200,
                                                      3);
  std::cout << "Decision threshold: " << decision_threshold << std::endl;
}

TEST(SPRTTest, SequentialProbabilityRatioTestPass) {
  // Create a set of points along y=x with a small random pertubation.
  vector<Point> input_points;
  for (int i = 0; i < 10000; ++i) {
    double noise_x = RandDouble(-1, 1);
    double noise_y = RandDouble(-1, 1);
    input_points.push_back(Point(i + noise_x, i + noise_y));
  }
  // Test for the correct line.
  Line fitting_line(1.0, 0.0);
  LineEstimator estimator;
  // Error threshold to consider points inliers vs outliers.
  double error_thresh = 0.5;
  // Estimate type 1 error.
  double sigma = 0.05;
  // Estimate of inlier ratio.
  double epsilon = 0.6;

  // Calculate the decision threshold.
  double decision_threshold = CalculateSPRTDecisionThreshold(sigma, epsilon);

  // Output parameters of SPRT.
  int num_tested_points;
  double observed_inlier_ratio;

  // Execute SPRT with a line we expect to fit the data.
  bool sprt_success = SequentialProbabilityRatioTest(input_points,
                                                     fitting_line,
                                                     estimator,
                                                     error_thresh,
                                                     sigma,
                                                     epsilon,
                                                     decision_threshold,
                                                     &num_tested_points,
                                                     &observed_inlier_ratio);
  EXPECT_TRUE(sprt_success);
}

TEST(SPRTTest, SequentialProbabilityRatioTestFail) {
  // Create a set of points along y=x with a small random pertubation.
  vector<Point> input_points;
  for (int i = 0; i < 10000; ++i) {
    double noise_x = RandDouble(-1, 1);
    double noise_y = RandDouble(-1, 1);
    input_points.push_back(Point(i + noise_x, i + noise_y));
  }
  // Test for the correct line.
  Line fitting_line(1.0, 0.0);
  LineEstimator estimator;
  // Error threshold to consider points inliers vs outliers.
  double error_thresh = 0.5;
  // Estimate type 1 error.
  double sigma = 0.05;
  // Estimate of inlier ratio.
  double epsilon = 0.6;

  // Calculate the decision threshold.
  double decision_threshold = CalculateSPRTDecisionThreshold(sigma, epsilon);

  // Output parameters of SPRT.
  int num_tested_points;
  double observed_inlier_ratio;

  // Execute SPRT with a few lines that do not fit the data.
  Line not_fitting_line(-1.0, 50);
  bool sprt_success = SequentialProbabilityRatioTest(input_points,
                                                     not_fitting_line,
                                                     estimator,
                                                     error_thresh,
                                                     sigma,
                                                     epsilon,
                                                     decision_threshold,
                                                     &num_tested_points,
                                                     &observed_inlier_ratio);
  EXPECT_FALSE(sprt_success);

  not_fitting_line = Line(1.0, 10);
  sprt_success = SequentialProbabilityRatioTest(input_points,
                                                not_fitting_line,
                                                estimator,
                                                error_thresh,
                                                sigma,
                                                epsilon,
                                                decision_threshold,
                                                &num_tested_points,
                                                &observed_inlier_ratio);
  EXPECT_FALSE(sprt_success);

  not_fitting_line = Line(2.0, 0);
  sprt_success = SequentialProbabilityRatioTest(input_points,
                                                not_fitting_line,
                                                estimator,
                                                error_thresh,
                                                sigma,
                                                epsilon,
                                                decision_threshold,
                                                &num_tested_points,
                                                &observed_inlier_ratio);
  EXPECT_FALSE(sprt_success);
}
}  // namespace solvers
