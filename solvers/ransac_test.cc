// Copyright (C) 2013  Chris Sweeney <cmsweeney@cs.ucsb.edu>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
//
//     * Neither the name of the University of California, Santa Barbara nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL CHRIS SWEENEY BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include <math.h>

#include "gtest/gtest.h"
#include "solvers/estimator.h"
#include "solvers/ransac.h"

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

TEST(RansacTest, NoInput) {
  LineEstimator line_estimator;
  Line line;
  vector<Point> input_points;
  Ransac<Point, Line> ransac_line(2, 0.7, 10000);
  bool success = ransac_line.Compute(input_points, line_estimator, 0.3, &line);
  ASSERT_FALSE(success);
}

TEST(RansacTest, LineFitting) {
  // Create a set of points along y=x with a small random pertubation.
  vector<Point> input_points;
  for (int i = 0; i < 10000; ++i) {
    double noise_x = RandDouble(-1.0, 1.0);
    double noise_y = RandDouble(-1.0, 1.0);
    input_points.push_back(Point(i + noise_x, i + noise_y));
  }

  LineEstimator line_estimator;
  Line line;
  Ransac<Point, Line> ransac_line(2, 0.7, 10000);
  ransac_line.Compute(input_points, line_estimator, 0.3, &line);
  ASSERT_LT(fabs(line.m - 1.0), 0.1);
}

TEST(RansacTest, GetInliers) {
  // Create a set of points along y=x with a small random pertubation.
  vector<Point> input_points;
  for (int i = 0; i < 10000; ++i) {
    double noise_x = RandDouble(-1, 1);
    double noise_y = RandDouble(-1, 1);
    input_points.push_back(Point(i + noise_x, i + noise_y));
  }

  LineEstimator line_estimator;
  Line line;
  double error_thresh = 0.3;
  // Set an un-obtainable value for the inlier ratio (e.g. > 1.0)
  Ransac<Point, Line> ransac_line(2, 0.7, 10000);
  ransac_line.Compute(input_points, line_estimator, error_thresh, &line);

  // Ensure each inlier is actually an inlier.
  vector<bool> inliers = ransac_line.GetInliers();
  for (int i = 0; i < input_points.size(); i++) {
    bool verified_inlier =
        line_estimator.Error(input_points[i], line) < error_thresh;
    ASSERT_EQ(verified_inlier, inliers[i]);
  }
}

TEST(RansacTest, TerminationNumInliers) {
  // Create a set of points along y=x with a small random pertubation.
  vector<Point> input_points;
  for (int i = 0; i < 10000; ++i) {
    double noise_x = RandDouble(-1, 1);
    double noise_y = RandDouble(-1, 1);
    input_points.push_back(Point(i + noise_x, i + noise_y));
  }

  LineEstimator line_estimator;
  Line line;
  // Set an un-obtainable value for the inlier ratio (e.g. > 1.0)
  Ransac<Point, Line> ransac_line(2, 2.0, 10000);
  ransac_line.termination_num_inliers = 10;
  ransac_line.Compute(input_points, line_estimator, 0.3, &line);

  int num_inliers = ransac_line.GetNumInliers();
  ASSERT_GE(num_inliers, 10);
}

}  // namespace solvers
