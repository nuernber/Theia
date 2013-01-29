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

#include "math/polynomial.h"

#include <algorithm>
#include <Eigen/Dense>
#include <iostream>
#include <glog/logging.h>
#include <tuple>
#include <utility>
#include <vector>

namespace math {
namespace {
struct SturmInterval {
  double lower_bound;
  int lower_val;
  double upper_bound;
  int upper_val;
  SturmInterval(double lb, int lv, double ub, int uv)
      : lower_bound(lb), lower_val(lv), upper_bound(ub), upper_val(uv) {}
};

inline bool SignOf(const double& d ) {
  return d < 0.0;
}

class SturmChain {
 public:
  explicit SturmChain(const Polynomial& poly) {
    q_.resize(poly.GetDegree() + 1);

    // Create Sturm Chain.
    coeffs_ = Polynomial(poly);
    derivitive_ = coeffs_.Differentiate();

    Polynomial f_i_plus_2 = coeffs_;
    Polynomial f_i_plus_1 = derivitive_;
    Polynomial f_current;
    for (int i = q_.size() - 3; i >= 0; i--) {
      std::tie(q_[i+2], f_current) =
          f_i_plus_2.Divide(f_i_plus_1);
      f_i_plus_2 = f_i_plus_1.Trim();
      f_i_plus_1 = -f_current.Trim();
    }
    q_[1] = f_i_plus_2;
    q_[0] = f_i_plus_1;
  }
  ~SturmChain() {}

  // Returns the stum chain evaluation, i.e. number of sign changes observed.
  int EvalAt(double val) const {
    int sign_changes = 0;

    double f_minus_2 = q_[0].EvalAt(val);
    double f_minus_1 = q_[1].EvalAt(val);
    // If different signs.
    if (SignOf(f_minus_2) != SignOf(f_minus_1))
      sign_changes++;

    for (int i = 2; i < q_.size(); i++) {
      // Evaluate poly at current sturm chain term.
      double f_current = q_[i].EvalAt(val)*f_minus_1 - f_minus_2;
      // If there was a sign change.
      if (SignOf(f_current) != SignOf(f_minus_1))
        sign_changes++;
      f_minus_2 = f_minus_1;
      f_minus_1 = f_current;
    }
    return sign_changes;
  }

  // Sturm chain evaluation at infinity.
  int EvalAtInfinity() const {
    int sign_changes = 0;
    bool f_minus_2 = SignOf(q_[0][0]);
    bool f_minus_1 = SignOf(q_[1][1]);
    // If different signs.
    if (f_minus_2 != f_minus_1)
      sign_changes++;

    for (int i = 2; i < q_.size(); i++) {
      // Evaluate poly at current sturm chain term. Only the sign of the second
      // term matters.
      bool f_current = f_minus_1 != SignOf(q_[i][1]);
      // If there was a sign change at infinity.
      if (f_current != f_minus_1)
        sign_changes++;
      f_minus_1 = f_current;
    }
    return sign_changes;
  }

  // Sturm chain evalution at negative infinity.
  int EvalAtNegInfinity() const {
    int sign_changes = 0;
    bool f_minus_2 = SignOf(q_[0][0]);
    bool f_minus_1 = !SignOf(q_[1][1]);
    // If different signs.
    if (f_minus_2 != f_minus_1)
      sign_changes++;

    for (int i = 2; i < q_.size(); i++) {
      // Evaluate poly at current sturm chain term. Only the sign of the second
      // term matters.
      bool f_current = f_minus_1 != !SignOf(q_[i][1]);
      // If there was a sign change at infinity.
      if (f_current != f_minus_1)
        sign_changes++;
      f_minus_1 = f_current;
    }
    return sign_changes;
  }

  // Evaluate the polynomial at x.
  double PolyEval(double x) const {
    return coeffs_.EvalAt(x);
  }

  // Evaluate the derivitive at x.
  double DerivEval(double x) const {
    return derivitive_.EvalAt(x);
  }

 private:
  // Array of polynomials f0, ..., fn
  Polynomial coeffs_;
  Polynomial derivitive_;
  // Array of quotients of the form mx + b (m and x are saved).
  std::vector<Polynomial> q_;
};

double BisectionRefineRoot(const SturmChain& sturm,
                           double lower,
                           double lower_val,
                           double upper,
                           double upper_val) {
  double kMaxIterations = 60;
  double kEps = 1e-9;

  // Only puruse a constant number of iterations of bisection.
  double midpoint = (upper + lower)/2.0;
  double midpoint_val = sturm.PolyEval(midpoint);
  for (int i = 0; fabs(midpoint_val) > kEps && i < kMaxIterations; i++) {
    // If the root lays in the lower half, use that for futher bisectioning.
    if (SignOf(midpoint_val) != SignOf(lower_val)) {
      upper = midpoint;
      upper_val = midpoint_val;
    } else {
      lower = midpoint;
      lower_val = midpoint_val;
    }
    midpoint = (upper + lower)/2.0;
    midpoint_val = sturm.PolyEval(midpoint);
  }
  return midpoint;
}

double NewtonRefineRoot(const SturmChain& sturm,
                        double lower,
                        double lower_val,
                        double upper,
                        double upper_val) {
  double kMaxIterations = 12;
  double kEps = 1e-16;

  // Initial guess.
  double x = (lower + upper)/2.0;
  double x_prev = upper;
  int iter = 0;
  // Update according to Taylor series expansion.
  for (int i = 0; fabs(x - x_prev) > kEps && i < kMaxIterations; i++) {
    x_prev = x;
    x = x_prev - sturm.PolyEval(x_prev)/sturm.DerivEval(x_prev);
    if (x < lower)
      x = (lower + x_prev)/2.0;
    else if (x > upper)
      x = (upper + x_prev)/2.0;
  }

  return x;
}

// Recursive function to find the roots. Bounds are the lower and upper bounds
// of the interval, and val are the evaluation of the sturm chain's changes at
// the respective bounds (i.e. lower_bound_val - upper_bound_val = number of
// roots in interval).
void FindSturmRoots(const SturmChain& sturm,
                    double lower_bound,
                    int lower_bound_val,
                    double upper_bound,
                    int upper_bound_val,
                    std::vector<double>* roots) {
  // If there is only one root in the bound, refine it using bisection.
  if (lower_bound_val - upper_bound_val == 1) {
    roots->push_back(NewtonRefineRoot(sturm,
                                      lower_bound,
                                      sturm.PolyEval(lower_bound),
                                      upper_bound,
                                      sturm.PolyEval(upper_bound)));
  } else {
    double midpoint = (upper_bound + lower_bound)/2.0;
    int midpoint_val = sturm.EvalAt(midpoint);
    // If mid and lower are not equal then root(s) must exist within those
    // bounds.
    if (midpoint_val != lower_bound_val) {
      FindSturmRoots(sturm,
                     lower_bound,
                     lower_bound_val,
                     midpoint,
                     midpoint_val,
                     roots);
    }
    // If mid and upper are not equal then root(s) must exist within those
    // bounds.
    if (midpoint_val != upper_bound_val) {
      FindSturmRoots(sturm,
                     midpoint,
                     midpoint_val,
                     upper_bound,
                     upper_bound_val,
                     roots);
    }
  }
}

}  // namespace


// coeffs is an array containing the coefficients of the polynomial, with
// coeffs[i] corresponding to the i-th degree coefficient.
Polynomial::Polynomial(int degree, const double coeffs[]) {
  coeffs_.resize(degree + 1);
  coeffs_.assign(coeffs, coeffs + degree + 1);
}

// Resize the polynomial so there are no leading zeros.
Polynomial& Polynomial::Trim() {
  int i;
  for (i = coeffs_.size() - 1; coeffs_[i] == 0.0 && i >= 0; i--);
  if (i < coeffs_.size() - 1)
    coeffs_.resize(i + 1);
  return *this;
}

// Evaluate the polynomial at x.
double Polynomial::EvalAt(const double x) const {
  double val = 0;
  for (int i = coeffs_.size() - 1; i > 0; i--) {
    val += coeffs_[i];
    val *= x;
  }
  val += coeffs_[0];
  return val;
}

// Add two polynomials together.
Polynomial Polynomial::Add(const Polynomial& poly) const {
  std::vector<double> sum(std::max(poly.coeffs_.size(), coeffs_.size()));
  for (int i = 0; i < poly.coeffs_.size(); i++)
    sum[i] += poly.coeffs_[i];
  for (int i = 0; i < coeffs_.size(); i++)
    sum[i] += coeffs_[i];
  return Polynomial(sum);
}

// Subtract such that ret_poly = this_poly - poly.
Polynomial Polynomial::Subtract(const Polynomial& poly) const {
  std::vector<double> diff(std::max(poly.coeffs_.size(), coeffs_.size()));
  for (int i = 0; i < poly.coeffs_.size(); i++)
    diff[i] -= poly.coeffs_[i];
  for (int i = 0; i < coeffs_.size(); i++)
    diff[i] += coeffs_[i];
  return Polynomial(diff);
}

Polynomial Polynomial::Multiply(const Polynomial& poly) const {
  std::vector<double> ret(coeffs_.size() + poly.coeffs_.size() - 1);
  for (int i = 0; i < coeffs_.size(); i++)
    for (int j = 0; j < poly.coeffs_.size(); j++)
      ret[i + j] += coeffs_[i]*poly.coeffs_[j];
  return Polynomial(ret);
}

// Returns the quotient and the remainder as separate polynomials.
std::pair<Polynomial, Polynomial> Polynomial::Divide(
    const Polynomial& poly) const {
  std::vector<double> quotient(coeffs_.size() - poly.coeffs_.size() + 1);
  std::vector<double> remainder(coeffs_);

  for (int i = quotient.size() - 1; i >= 0; i--) {
    double scale = remainder[i + poly.coeffs_.size() - 1]/
        poly.coeffs_[poly.coeffs_.size() - 1];
    quotient[i] = scale;
    for (int j = 0; j < poly.coeffs_.size(); j++)
      remainder[i + j] -= scale*poly.coeffs_[j];
  }
  remainder.resize(poly.coeffs_.size() - 1);
  return std::make_pair(Polynomial(quotient), Polynomial(remainder));
}

Polynomial Polynomial::Differentiate() const {
  std::vector<double> ret(coeffs_.size() - 1);
  for (int i = 1; i < coeffs_.size(); i++)
    ret[i-1] = coeffs_[i] * i;

  return Polynomial(ret);
}

Polynomial Polynomial::operator -() {
  std::vector<double> negated(coeffs_.size());
  for (int i = 0; i < coeffs_.size(); i++)
    negated[i] = -1.0*coeffs_[i];
  return Polynomial(negated);
}

std::vector<double> Polynomial::RealRoots() const {
  std::vector<std::complex<double> > complex_roots = Roots();
  std::vector<double> roots;
  for (int i = 0; i < complex_roots.size(); i++)
    if(complex_roots[i].imag() == 0.0)
      roots.push_back(complex_roots[i].real());
  return roots;
}

std::vector<std::complex<double> > Polynomial::Roots() const {
  const int degree = coeffs_.size() - 1;
  Eigen::Map<const Eigen::VectorXd> poly(&coeffs_[0], degree + 1, 1);
  Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(degree, degree);

  // Construct companion matrix.
  companion.row(0) = -1.0*poly.head(degree).reverse();
  companion.row(0) /= poly(degree);
  for (int i = 1; i < degree; i++)
    companion(i, i-1) = 1.0;
  // Compute eigenvalues. Note, these can be complex.
  Eigen::VectorXcd eigenvalues = companion.eigenvalues();
  std::vector<std::complex<double> > roots(eigenvalues.rows());
  for (int i = 0; i < roots.size(); i++)
    roots[i] = eigenvalues(i);
  return roots;
}

// Solve for real roots using Sturm Chain.
std::vector<double> Polynomial::RealRootsSturm() const {
  // Initialize Sturm Chain.
  SturmChain sturm(*this);

  // Evaluate sturm at -infinity, infinity to determine total # of roots.
  int positive_sturm_eval = sturm.EvalAtInfinity();
  int negative_sturm_eval = sturm.EvalAtNegInfinity();
  int num_roots = negative_sturm_eval - positive_sturm_eval;

  // Try to find an upper and lower bound (start at 1, -1 then multiply by 2)
  // Roots above 100 are so steep that machine precision can lead to massive
  // inaccuracies, so we cap it here.
  double highest_root = 100.0;
  double lower_bound = -1.0;
  while (negative_sturm_eval - sturm.EvalAt(lower_bound) != 0 &&
         lower_bound >= -highest_root)
    lower_bound *= 2.0;

  double upper_bound = 1.0;
  while (sturm.EvalAt(upper_bound) - positive_sturm_eval != 0 &&
         upper_bound <= highest_root)
    upper_bound *= 2.0;

  // Initiate Recursive call.
  std::vector<double> roots;
  FindSturmRoots(sturm,
                 lower_bound,
                 sturm.EvalAt(lower_bound),
                 upper_bound,
                 sturm.EvalAt(upper_bound),
                 &roots);
  return roots;
}

}  // namespace math
