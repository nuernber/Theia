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

#include "math/closed_form_polynomial_solver.h"

#include <glog/logging.h>

#include <complex>
#include <cmath>
using std::complex;

// Number within the error threshold of zero.
#define kEpsilon 1e-12

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
    if (std::abs(complex_roots[i].imag()) < kEpsilon)
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
    if (std::abs(complex_roots[i].imag()) < kEpsilon)
      roots[num_real_solutions++] = complex_roots[i].real();
  }
  return num_real_solutions;
}

int SolveCubic(double a, double b, double c, double d, complex<double>* roots) {
  if (a == 0.0)
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
    VLOG(0) << "sol = " << complex_roots[i].real() << ", "
            << complex_roots[i].imag();
    if (std::abs(complex_roots[i].imag()) < kEpsilon) {
      roots[num_real_solutions++] = complex_roots[i].real();
    }
  }
  return num_real_solutions;
}

int SolveQuartic(double a, double b, double c, double d, double e,
                 complex<double>* roots) {
  using std::pow;
  using std::sqrt;

  double a_pw2 = a*a;
  double b_pw2 = b*b;
  double a_pw3 = a_pw2*a;
  double b_pw3 = b_pw2*b;
  double a_pw4 = a_pw3*a;
  double b_pw4 = b_pw3*b;

  double alpha = -3*b_pw2/(8*a_pw2)+c/a;
  double beta = b_pw3/(8*a_pw3)-b*c/(2*a_pw2)+d/a;
  double gamma = -3*b_pw4/(256*a_pw4)+b_pw2*c/(16*a_pw3)-b*d/(4*a_pw2)+e/a;

  double alpha_pw2 = alpha*alpha;
  double alpha_pw3 = alpha_pw2*alpha;

  complex<double> P(-alpha_pw2/12 - gamma, 0);
  complex<double> Q(-alpha_pw3/108 + alpha*gamma/3 - pow(beta, 2)/8, 0);
  complex<double> R = -Q/2.0 + sqrt(pow(Q, 2)/4.0+pow(P, 3.0)/27.0);

  complex<double> U = pow(R, (1.0/3.0));
  complex<double> y;

  if (U.real() == 0)
    y = -5.0*alpha/6.0 - pow(Q, (1.0/3.0));
  else
    y = -5.0*alpha/6.0 - P/(3.0*U) + U;

  complex<double> w = sqrt(alpha + 2.0*y);

  complex<double> temp;
  VLOG(0) << "a = " << a
          << "\nb = " << b
          << "\nc = " << c
          << "\nd = " << d
          << "\ne = " << e;
  roots[0] = -b/(4.0*a) + 0.5*(w + sqrt(-(3.0*alpha + 2.0*y + 2.0*beta/w)));
  roots[1] = -b/(4.0*a) + 0.5*(w - sqrt(-(3.0*alpha + 2.0*y + 2.0*beta/w)));
  roots[2] = -b/(4.0*a) + 0.5*(-w + sqrt(-(3.0*alpha + 2.0*y - 2.0*beta/w)));
  roots[3] = -b/(4.0*a) + 0.5*(-w - sqrt(-(3.0*alpha + 2.0*y - 2.0*beta/w)));

  return 4;
}
}  // namespace math
