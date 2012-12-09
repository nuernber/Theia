#include "solvers/closed_form_polynomial_solver.h"

#include <complex>
#include <iostream>
using std::complex;

// Number within the error threshold of zero.
#define kEpsilon 1e-6

namespace solvers {
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
    if (complex_roots[i].imag() < kEpsilon)
      roots[num_real_solutions++] = complex_roots[i].real();
  }
  return num_real_solutions;
}

// Using Ferrari's method of solution for general quartics (Wikipedia).
// http://en.wikipedia.org/wiki/Quartic_function#Summary_of_Ferrari.27s_method
int SolveQuartic(double a, double b, double c, double d, double e,
                 complex<double>* roots) {
  using std::pow;
  using std::sqrt;
  double alpha = (-3.0*b*b)/(8.0*a*a) + c/a;
  std::cout << "alpha = " << alpha << std::endl;
  double beta = (b*b*b)/(8.0*a*a*a) - (b*c)/(2.0*a*a) + d/a;
  std::cout << "beta = " << beta << std::endl;
  double gamma = (-3.0*b*b*b*b)/(256.0*a*a*a*a) + (c*b*b)/(16.0*a*a*a) -
      (b*d)/(4.0*a*a) + e/a;
  std::cout << "gamma = " << gamma << std::endl;

  if (beta == 0.0) {
    double tmp = alpha*alpha - 4.0*gamma;
    complex<double> sqrt_val = sqrt(tmp);
    roots[0] = (-1.0*b)/(4.0*a) + sqrt((-1.0*alpha + sqrt_val)/2.0);
    roots[1] = (-1.0*b)/(4.0*a) + sqrt((-1.0*alpha - sqrt_val)/2.0);
    roots[2] = (-1.0*b)/(4.0*a) - sqrt((-1.0*alpha + sqrt_val)/2.0);
    roots[3] = (-1.0*b)/(4.0*a) - sqrt((-1.0*alpha - sqrt_val)/2.0);
    return 4;
  }
  
  double p = (-1.0*alpha*alpha)/12.0 - gamma;
  double q = (-1.0*alpha*alpha*alpha)/108.0 + (alpha*gamma)/3.0 -
      (beta*beta)/8.0;

  double r_temp = (q*q)/4.0 + (p*p*p)/27.0;
  // Either sign of the sqrt of r will be sufficient (we choose +).
  complex<double> r = (-1.0*q)/2.0 + sqrt(r_temp);
  complex<double> u = pow(r, 1.0/3.0);
  complex<double> y = (-5.0*alpha)/6.0 + u;
  if (abs(u) < kEpsilon)
    y -= p/(3.0*u);
  else
    y -= pow(q, 1.0/3.0);

  complex<double> w = sqrt(alpha + 2.0*y);
  
  complex<double> temp_plus = -1.0*(3.0*alpha + 2.0*y + (2.0*beta)/w);
  complex<double> sqrt_plus = sqrt(temp_plus);
  complex<double> temp_minus = -1.0*(3.0*alpha + 2.0*y - (2.0*beta)/w);
  complex<double> sqrt_minus = sqrt(temp_minus);

  roots[0] = (-1.0*b)/(4.0*a) + (w + sqrt_plus)/2.0;
  roots[1] = (-1.0*b)/(4.0*a) + (w - sqrt_plus)/2.0;
  roots[2] = (-1.0*b)/(4.0*a) + (-1.0*w + sqrt_minus)/2.0;
  roots[3] = (-1.0*b)/(4.0*a) + (-1.0*w + sqrt_minus)/2.0;
  return 4;
}
}  // namespace solvers
