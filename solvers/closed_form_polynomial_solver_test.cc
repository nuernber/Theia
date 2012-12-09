#include <complex>

#include "gtest/gtest.h"
#include "solvers/closed_form_polynomial_solver.h"

using std::complex;

namespace solvers {

TEST(SolveQuadraticPolynomial, DegenerateSolution) {
  // - 2x + 1 = 0
  double a = 0.0;
  double b = -2.0;
  double c = 1.0;
  double roots[2];
  int num_roots = SolveQuadraticReals(a, b, c, roots);
  EXPECT_EQ(num_roots, 1);
  EXPECT_EQ(roots[0], 0.5);
}

TEST(SolveQuadraticPolynomial, SolveReals) {
  // x^2 - 2x + 1 = 0
  double a = 1.0;
  double b = -2.0;
  double c = 1.0;
  double roots[2];
  int num_roots = SolveQuadraticReals(a, b, c, roots);
  EXPECT_EQ(num_roots, 2);
  EXPECT_EQ(roots[0], 1.0);

  // x^2 - 11x + 30 = 0
  a = 1.0;
  b = -11.0;
  c = 30.0;
  num_roots = SolveQuadraticReals(a, b, c, roots);
  EXPECT_EQ(num_roots, 2);
  EXPECT_NE(roots[0], roots[1]);
  EXPECT_TRUE(roots[0] == 5.0 || roots[1] == 5.0);
  EXPECT_TRUE(roots[0] == 6.0 || roots[1] == 6.0);
}

TEST(SolveQuadraticPolynomial, SolveComplex) {
  // x^2 - 2x + 5 = 0 should yield 1 + 2i, 1 - 2i
  double a = 1.0;
  double b = -2.0;
  double c = 5.0;
  complex<double> roots[2];
  int num_roots = SolveQuadratic(a, b, c, roots);
  EXPECT_EQ(num_roots, 2);
  EXPECT_EQ(roots[0].real(), 1.0);
  EXPECT_EQ(roots[1].real(), 1.0);
  EXPECT_TRUE(roots[0].imag() == 2.0 || roots[0].imag() == -2.0);
  EXPECT_TRUE(roots[1].imag() == 2.0 || roots[1].imag() == -2.0);
}

TEST(SolveCubicPolynomial, test1) {
  // x^3 - 6x^2 + 11x - 6 = 0
  double a = 1.0;
  double b = -6.0;
  double c = 11.0;
  double d = -6.0;
  double roots[3];
  int num_roots = SolveCubicReals(a, b, c, d, roots);
  std::cout << "num roots: " << num_roots << std::endl;
  for (int i = 0; i < 3; i++)
    std::cout << roots[i] << std::endl;
}

TEST(SolveQuarticPolynomial, test1) {
  double a = 3.0;
  double b = 6.0;
  double c = -123.0;
  double d = -126.0;
  double e = 1080.0;

  double roots[4];
  int num_roots = SolveQuarticReals(a, b, c, d, e, roots);
  std::cout << "num roots: " << num_roots << std::endl;
  for (int i = 0; i < 4; i++)
    std::cout << roots[i] << std::endl;
}
}  // namespace solvers
