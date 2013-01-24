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
#include <vector>

#include "gtest/gtest.h"
#include "solvers/arrsac.h"
#include "solvers/estimator.h"

using std::vector;

namespace solvers {
namespace {

// Create a testable instance of ARRSAC (i.e. move protected methods to public
// so that we can easily test it).
template<class Datum, class Model>
class TestableArrsac : public Arrsac<Datum, Model> {
 public:
  TestableArrsac(int min_sample_size,
                 int max_candidate_hyps = 500,
                 int block_size = 100)
      : Arrsac<Datum, Model>(min_sample_size, max_candidate_hyps, block_size) {}
  using Arrsac<Datum, Model>::GenerateInitialHypothesisSet;
};

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

TEST(ArrsacTest, InitializeHypothesisSet) {
  // Create a set of points along y=x with a small random pertubation.
  vector<Point> input_points;
  for (int i = 0; i < 10000; ++i) {
    double noise_x = RandDouble(-1.0, 1.0);
    double noise_y = RandDouble(-1.0, 1.0);
    input_points.push_back(Point(i + noise_x, i + noise_y));
  }
  vector<double> input_quality(input_points.size(), 0.0);

  LineEstimator estimator;
  // Error threshold to consider points inliers vs outliers.
  double error_thresh = 0.5;
  // Estimate type 1 error.
  double sigma = 0.05;
  // Estimate of inlier ratio.
  double epsilon = 0.6;

  vector<Line> initial_hypothesis;
  TestableArrsac<Point, Line> arrsac_line(2);
  int num_iterations =
      arrsac_line.GenerateInitialHypothesisSet(input_points,
                                               input_quality,
                                               estimator,
                                               error_thresh,
                                               &initial_hypothesis);
  ASSERT_GT(num_iterations, 0);
}

TEST(ArrsacTest, Compute) {
  // Create a set of points along y=x with a small random pertubation.
  vector<Point> input_points;
  for (int i = 0; i < 10000; ++i) {
    double noise_x = RandDouble(-1.0, 1.0);
    double noise_y = RandDouble(-1.0, 1.0);
    input_points.push_back(Point(i + noise_x, i + noise_y));
  }
  vector<double> input_quality(input_points.size(), 0.0);

  LineEstimator estimator;
  // Error threshold to consider points inliers vs outliers.
  double error_thresh = 0.3;
  // Estimate type 1 error.
  double sigma = 0.05;
  // Estimate of inlier ratio.
  double epsilon = 0.8;

  Line fitted_line;
  TestableArrsac<Point, Line> arrsac_line(2);
  bool success = arrsac_line.Compute(input_points,
                                     input_quality,
                                     estimator,
                                     error_thresh,
                                     &fitted_line);
  std::cout << "m = " << fitted_line.m << " b = " << fitted_line.b << std::endl;
}


TEST(ArrsacTest, ComputeWithQuality) {
  // Create a set of points along y=x with a small random pertubation.
  vector<Point> input_points;
  for (int i = 0; i < 10000; ++i) {
    double noise_x = RandDouble(-1.0, 1.0);
    double noise_y = RandDouble(-1.0, 1.0);
    input_points.push_back(Point(i + noise_x, i + noise_y));
  }
  vector<double> input_quality(input_points.size(), 0.0);

  // Set the several values to be without noise. Set the quality of these points
  // to 1 (all others are 0).
  input_points[0] = Point(0.0, 0.0);
  input_quality[0] = 1.0;
  input_points[1234] = Point(1234.0, 1234.0);
  input_quality[1234] = 1.0;
  input_points[5678] = Point(5768.0, 5678.0);
  input_quality[5678] = 1.0;

  LineEstimator estimator;
  // Error threshold to consider points inliers vs outliers.
  double error_thresh = 0.3;
  // Estimate type 1 error.
  double sigma = 0.05;
  // Estimate of inlier ratio.
  double epsilon = 0.8;

  Line fitted_line;
  TestableArrsac<Point, Line> arrsac_line(2);
  bool success = arrsac_line.Compute(input_points,
                                     input_quality,
                                     estimator,
                                     error_thresh,
                                     &fitted_line);
  std::cout << "m = " << fitted_line.m << " b = " << fitted_line.b << std::endl;
}

}  // namespace solvers
