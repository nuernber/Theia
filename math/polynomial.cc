#include "math/polynomial.h"

#include <algorithm>
#include <stack>
#include <tuple>
#include <utility>
#include <vector>

namespace math {
namespace {
inline bool SignOf(const double& d ){
  return d < 0.0;
}

class SturmChain {
 public:
  explicit SturmChain(const Polynomial& poly) {
    q_.resize(poly.GetDegree() + 1);

    // Create Sturm Chain.
    std::vector<Polynomial> f_(poly.GetDegree() + 1);
    coeffs_ = Polynomial(poly);
    derivitive_ = coeffs_.Differentiate();

    Polynomial f_i_plus_2 = coeffs_;
    Polynomial f_i_plus_1 = derivitive_;
    Polynomial f_current;
    for (int i = f_.size() - 3; i >= 0; i--) {
      std::tie(q_[i+2], f_current) =
          f_i_plus_2.Divide(f_i_plus_1);
      f_i_plus_2 = f_i_plus_1;
      f_i_plus_1 = -f_current;
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

// Evaluate the polynomial at x.
double Polynomial::EvalAt(const double x) const {
  double val = 0;
  for(int i = coeffs_.size() - 1; i > 0; i--) {
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
  for(int i = 0; i < coeffs_.size(); i++)
    for(int j = 0; j < poly.coeffs_.size(); j++)
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
  for(int i = 1; i < coeffs_.size(); i++)
    ret[i-1] = coeffs_[i] * i;

  return Polynomial(ret);
}

Polynomial Polynomial::operator -() {
  std::vector<double> negated(coeffs_.size());
  for (int i = 0; i < coeffs_.size(); i++)
    negated[i] = -1.0*coeffs_[i];
  return Polynomial(negated);
}

// Solve for real roots using Sturm Chain.
std::vector<double> Polynomial::RealRoots() const {
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
