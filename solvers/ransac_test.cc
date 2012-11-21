#include "ransac.h"

#include "gtest/gtest.h"
#include <math.h>

using namespace std;

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
  LineEstimator() {};
  ~LineEstimator() {};

  bool EstimateModel(const vector<Point>& data, Line* model) {
    model->m = (data[1].y - data[0].y)/(data[1].x - data[0].x);
    model->b = data[1].y - model->m*data[1].x;
    return true;
  }
  
  double Error(const Point& point, const Line& line) {
    double a = -1.0*line.m;
    double b = 1.0;
    double c = -1.0*line.b;

    return fabs(a*point.x + b*point.y + c)/(sqrt(pow(a*a + b*b, 2)));
  }
};

// Returns a random double between dMin and dMax
double RandDouble(double dMin, double dMax)
{
    double d = (double)rand() / RAND_MAX;
    return dMin + d * (dMax - dMin);
}
}  // namespace

TEST(RansacTest, NoInput) {
  LineEstimator line_estimator;
  Line line;
  vector<Point> input_points;
  Ransac<Point, Line> ransac_line(2, 0.7, 10000);
  bool success = ransac_line.Compute(input_points, 0.3, &line_estimator, &line);
  ASSERT_FALSE(success);
}

TEST(RansacTest, LineFitting) {
  // Create a set of points along y=x with a small random pertubation.
  vector<Point> input_points;
  for (int i = 0; i < 100; ++i) {
    double noise_x = RandDouble(-1,1);
    double noise_y = RandDouble(-1,1);
    input_points.push_back(Point(i + noise_x, i + noise_y));
  }

  LineEstimator line_estimator;
  Line line;
  Ransac<Point, Line> ransac_line(2, 0.7, 10000);
  ransac_line.Compute(input_points, 0.3, &line_estimator, &line);
  ASSERT_LT(fabs(line.m - 1.0), 0.1);
}

}  // namespace solvers
