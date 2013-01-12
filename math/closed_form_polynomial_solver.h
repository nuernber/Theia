#ifndef MATH_CLOSED_FORM_POLYNOMIAL_SOLVER_H_
#define MATH_CLOSED_FORM_POLYNOMIAL_SOLVER_H_

// These functions provides closed form solutions for n = {2, 3, 4} degree
// polynomials. Solve___Reals returns only the real solutions to the polynomial
// (i.e. solutions with no imaginary components). All methods return the number
// of solutions obtained and output the solutions in the roots array (for m
// solutions, the first m elements of the array will be filled with the roots).

#include <complex>

namespace math {
// Provides solutions to the equation a*x^2 + b*x + c = 0.
int SolveQuadraticReals(double a, double b, double c, double* roots);
int SolveQuadratic(double a, double b, double c, std::complex<double>* roots);

// Provides solutions to the equation a*x^3 + b*x^2 + c*x + d = 0 using Cardan's
// method.
int SolveCubicReals(double a, double b, double c, double d, double* roots);
int SolveCubic(double a, double b, double c, double d,
               std::complex<double>* roots);

// Provides solutions to the equation a*x^4 + b*x^3 + c*x^2 + d*x + e = 0 using
// Ferrari's method to reduce to problem to a cubic.
int SolveQuarticReals(double a, double b, double c, double d, double e,
                       double* roots);
int SolveQuartic(double a, double b, double c, double d, double e,
                  std::complex<double>* roots);
}  // math
#endif  // MATH_CLOSED_FORM_POLYNOMIAL_SOLVER_H_
