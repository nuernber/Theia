#include "math/closed_form_polynomial_solver.h"

#include <complex>
#include <cmath>
#include <iostream>
using std::complex;

// Number within the error threshold of zero.
#define kEpsilon 1e-6

namespace math {
namespace {
// Solve depressed cubic using Cardano's method.
int SolveDepressedCubic(double p, double q, complex<double>* roots) {
  using std::pow;
  using std::sqrt;
  if (p == 0.0) {
    roots[0] = pow(-1.0*q, 1.0/3.0);
    return 1;
  }

  complex<double> cubic_root_of_unity(-0.5, 0.5*sqrt(3.0));
  complex<double> temp = q*q/4.0 + p*p*p/27.0;
  complex<double> sqrt_t = sqrt(temp);
  complex<double> u = pow(-0.5*q + sqrt_t, 1.0/3.0);
  complex<double> v = pow(-0.5*q - sqrt_t, 1.0/3.0);
  roots[0] = u + v;
  roots[1] = u*cubic_root_of_unity + v*cubic_root_of_unity*cubic_root_of_unity;
  roots[2] = u*cubic_root_of_unity*cubic_root_of_unity + v*cubic_root_of_unity;
  return 3;
}
}  // namespace

// Provides solutions to the equation a*x^2 + b*x + c = 0.
int SolveQuadraticReals(double a, double b, double c, double* roots) {
  complex<double> complex_roots[2];
  int num_complex_solutions = SolveQuadratic(a, b, c, complex_roots);
  int num_real_solutions = 0;
  for (int i = 0; i < num_complex_solutions; i++) {
    if (complex_roots[i].imag() < kEpsilon)
      roots[num_real_solutions++] = complex_roots[i].real();
  }
  return num_real_solutions;
}

int SolveQuadratic(double a, double b, double c, complex<double>* roots) {
  // Sqrt version that handles complex numbers.
  using std::sqrt;
  // If the equation is actually linear.
  if (a == 0.0) {
    roots[0] = -1.0*c/b;
    return 1;
  }

  complex<double> tmp_complex_num(b*b - 4.0*a*c, 0.0);
  complex<double> complex_sqrt(sqrt(tmp_complex_num));
  roots[0] = (-1.0*b + complex_sqrt)/(2.0*a);
  roots[1] = (-1.0*b - complex_sqrt)/(2.0*a);
  return 2;
}
// Provides solutions to the equation a*x^3 + b*x^2 + c*x + d = 0 using Cardan's
// method.
int SolveCubicReals(double a, double b, double c, double d, double* roots) {
  complex<double> complex_roots[3];
  int num_complex_solutions = SolveCubic(a, b, c, d, complex_roots);
  int num_real_solutions = 0;
  for (int i = 0; i < num_complex_solutions; i++) {
    if (complex_roots[i].imag() < kEpsilon)
      roots[num_real_solutions++] = complex_roots[i].real();
  }
  return num_real_solutions;
}

int SolveCubic(double a, double b, double c, double d, complex<double>* roots) {
  if(a == 0.0)
    return SolveQuadratic(b, c, d, roots);

  // Solve by first reducing the problem to a depressed cubic.
  double p = (3.0*a*c - b*b)/(3.0*a*a);
  double q = (2.0*b*b*b - 9.0*a*b*c + 27.0*a*a*d)/(27.0*a*a*a);
  int num_solutions = SolveDepressedCubic(p, q, roots);
  // Transform solution back to normal params.
  roots[0] -= b/(3.0*a);
  roots[1] -= b/(3.0*a);
  roots[2] -= b/(3.0*a);
  return num_solutions;
}

// Provides solutions to the equation a*x^4 + b*x^3 + c*x^2 + d*x + e = 0 using
// Ferrari's method to reduce to problem to a depressed cubic.
int SolveQuarticReals(double a, double b, double c, double d, double e,
                      double* roots) {
  complex<double> complex_roots[4];
  int num_complex_solutions = SolveQuartic(a, b, c, d, e, complex_roots);
  int num_real_solutions = 0;
  for (int i = 0; i < num_complex_solutions; i++) {
    if (std::abs(complex_roots[i].imag()) < kEpsilon) {
      roots[num_real_solutions++] = complex_roots[i].real();
    }
  }
  return num_real_solutions;
}

// Solve with a cubic transformation.
// http://mathforum.org/dr.math/faq/faq.cubic.equations.html
int SolveQuartic(double a, double b, double c, double d, double e,
                 complex<double>* roots) {
  using std::pow;
  using std::sqrt;
  using std::abs;

  // Solve by transforming the equation in two ways. First, divide the quartic
  // by a so that the leading coefficient is now 1 (i.e. x^4 + a'x^3 + b'x^2 +
  // c'x + d' = 0). Next, substitute x = y - a'/4. We can relate this new
  // depressed quartic to an auxillary cubic equation, and solve that equation
  // directly.
  double a1 = b/a;
  double b1 = c/a;
  double c1 = d/a;
  double d1 = e/a;
  double e1 = b1 - (3.0*a1*a1)/8.0;
  double f1 = c1 + (a1*a1*a1)/8.0 - (a1*b1)/2.0;
  double g1 = d1 - (3.0*a1*a1*a1*a1)/256.0 + (a1*a1*b1)/16.0 - (a1*c1)/4.0;
  if (g1 == 0.0) {
    // Function is a really a cubic. Roots are the 3 cubic roots and x = -a1/4.
    int num_sols = SolveCubic(1.0, 0.0, e1, f1, roots);
    if (num_sols == 0)
      return 0;
    roots[0] -= a1/4.0;
    roots[1] -= a1/4.0;
    roots[2] -= a1/4.0;
    roots[3] = (-1.0*a1)/4.0;
    return 4;
  } else if (f1 == 0.0) {
    // The function is actually a quadratic on y^2
    complex<double> quad_roots[2];
    int num_sols = SolveQuadratic(1.0, e1, g1, quad_roots);
    if (num_sols = 0)
      return 0;
    roots[0] = sqrt(quad_roots[0]);
    roots[1] = -1.0*roots[0];
    roots[2] = sqrt(quad_roots[1]);
    roots[3] - -1.0*roots[1];
    roots[0] -= a1/4.0;
    roots[1] -= a1/4.0;
    roots[2] -= a1/4.0;
    roots[3] -= a1/4.0;
    return 4;
  } else {
    // Solve the cubic per normal and plug the solution back in to retrieve the
    // quartic roots.
    complex<double> cubic_roots[3];
    double coeff1 = e1/2.0;
    double coeff2 = (e1*e1 - 4.0*g1)/16.0;
    double coeff3 = (-1.0*f1*f1)/64.0;
    int num_sols = SolveCubic(1.0, coeff1, coeff2, coeff3, cubic_roots);
    if (num_sols = 0)
      return 0;
    complex<double> p = sqrt(cubic_roots[0]);
    complex<double> q = sqrt(cubic_roots[1]);
    complex<double> r = (-1.0*f1)/(8.0*p*q);
    roots[0] = p + q + r - a1/4.0;
    roots[1] = p - q - r - a1/4.0;
    roots[2] = -1.0*p + q - r - a1/4.0;
    roots[3] = -1.0*p - q + r - a1/4.0;
    return 4;
  }
}
}  // namespace math
