.. highlight:: c++

.. default-domain:: cpp

.. _documentation-math:

====
Math
====

At the root of computer vision is a heavy amount of math and probability. Theia contains various math functions implemented with a generic interface for ease of use.

You can include the Math module with the following line:

.. code-block:: c++

  #include <theia/math.h>

.. _section-closed_form_poly:

Closed Form Polynomial Solver
=============================

Many problems in vision rely on solving a polynomial quickly. For small degrees
(n <= 4) this can be done in closed form, making them exceptionally fast. We
have implemented solvers for these cases.

.. function:: int SolveQuadraticReals(double a, double b, double c, double* roots)

.. function:: int SolveQuadratic(double a, double b, double c, std::complex<double>* roots)

  Provides solutions to the equation :math:`a*x^2 + b*x + c = 0`

.. function::  int SolveCubicReals(double a, double b, double c, double d, double* roots)

.. function::  int SolveCubic(double a, double b, double c, double d, std::complex<double>* roots)

   Provides solutions to the equation :math:`a*x^3 + b*x^2 + c*x + d = 0` using `Cardano's <http://en.wikipedia.org/wiki/Cubic_function#Cardano.27s_method>`_ method.


.. function::  int SolveQuarticReals(double a, double b, double c, double d, double e, double* roots)

.. function::  int SolveQuartic(double a, double b, double c, double d, double e, std::complex<double>* roots)

  Provides solutions to the equation :math:`a*x^4 + b*x^3 + c*x^2 + d*x + e = 0` using `Ferrari's method <http://en.wikipedia.org/wiki/Quartic_function#Ferrari.27s_solution>`_ to reduce to problem to a depressed cubic.


.. _section-generic_poly:

Generic Polynomial Solver
=========================

For polynomials of degree > 4 there are no easy closed-form solutions, making the problem of finding roots much more difficult. However, we have implemented a generic :class:`Polynomial` class that is templated on the degree of the polynomial. Because it is templated, the methods are generated at compile-time, so solving the polynomial ends up being extremely fast compared to non-templated methods (with relative few tradeoffs in terms of usability).

.. class:: Polynomial

  .. code-block:: c++

    template<int degree>
    class Polynomial {
     public:
      explicit Polynomial(const std::vector<double> coeffs);
      explicit Polynomial(const double coeffs[]);
      ~Polynomial() {}

      int GetDegree() const { return degree; }
      double EvalAt(const double x) const;

      template<int degree2>
      Polynomial Add(const Polynomial<degree2>& poly) const;

      template<int degree2>
      Polynomial Subtract(const Polynomial<degree2>& poly) const;

      template<int degree2>
      Polynomial<degree + degree2> Multiply(const Polynomial<degree2>& poly) const;

      // Returns the quotient polynomial and the remainder polynomial.
      template<int degree2>
      std::pair<Polynomial<degree - degree2>, Polynomial<degree2 - 1> > Divide(
	  const Polynomial<degree2>& poly) const;

      Polynomial<degree - 1> Differentiate() const;
      std::vector<double> RealRoots() const;
      std::vector<std::complex<double> > Roots() const;
    };

  The :class:`Polynomial` class allows for all basic polynomial operations, as
  well as some more advances operations (e.g. polynomial division,
  differentiation, etc.). The functions are particular note are the roots functions:

  .. function:: std::vector<std::complex<double> > Roots() const

    These methods calculate the roots of the polynomial (up to roughly degree
    100) efficiently by building a companion matrix and solving for the
    eigenvalues. This method has been shown to be very efficient, and also takes
    advantages of the underlying data structure (Eigen3 matrix). In order to
    increase the stability, we first balance the companion matrix by rearranging
    and scaling so that the changes in magnitude amongst matrix entries is not
    drastic.

  .. function:: std::vector<double> RealRoots() const

    Same as the above method, but only returns the real roots. That is, only the
    roots that have complex values < epsilon distance from 0. Note, this is
    different than only returning the real values of the solutions
    (i.e. ignoring the imaginary component).


.. _section-gauss_jordan:

Guass-Jordan
============



.. function:: void GaussJordan(Eigen::MatrixBase<Derived>* input, int max_rows = 99999)

  Perform traditional Gauss-Jordan elimination on an Eigen3 matrix. If
  ``max_rows`` is specified, it will on perform Gauss-Jordan on the first
  ``max_rows`` number of rows. This is useful for problems where your system is
  extremely overdetermined and you do not need all rows to be solved.


.. _section_ks_test:

Kolmogorov-Smirnoff Two Sample Test
===================================

Sometimes it is useful to test if two distributions come from the same distribution. This is particularly the case in RANSAC-like methods where inliers, for instance, are known to behave according to a chi-squared distribution. The `Kolmogorov-Smirnoff Test <http://en.wikipedia.org/wiki/Kolmogorov-Smirnov_test>`_ determines whether two sets of data come from the same distribution by inspecting the empirical distribution functions of the two sets. We have only implemented the two-sample Kolmogorov-Smirnoff test.

.. function:: bool KolmogorovSmirnoffTest(const std::vector<double>& residual1, const std::vector<double>& residual2)

.. function:: bool KolmogorovSmirnoffTest(const std::vector<double>& residual1, const std::vector<double>& residual2, int n)

  ``n``: Only performs the KS-Test for the first n values of each set.

.. function:: bool KolmogorovSmirnoffTest(const std::vector<double>& residual1, const std::vector<double>& residual2, int n1, int n2)

  ``n1, n2`` Only performs the KS-Test for the first n1 members of ``residual1`` and the first n2 members of ``residual2``.


.. _section-sprt:

Sequential Probability Ratio Test
=================================

Modified version of Wald's `SPRT <http://en.wikipedia.org/wiki/Sequential_probability_ratio_test>`_ as [Matas]_ et. al. implement it in "Randomized
RANSAC with Sequential Probability Ratio Test"

.. function:: double CalculateSPRTDecisionThreshold(double sigma, double epsilon, double time_compute_model_ratio = 200.0, int num_models_verified = 1)

 ``sigma``: Probability of rejecting a good model (Bernoulli parameter).

 ``epsilon``: Inlier ratio.

 ``time_compute_model_ratio``: Computing the model parameters from a sample takes the same time as verification of time_compute_model_ratio data points. Matas et. al. use 200.

 ``num_model_verified``: Number of models that are verified per sample.

 ``Returns``:  The SPRT decision threshold based on the input parameters.


.. function:: bool SequentialProbabilityRatioTest(const std::vector<double>& residuals, double error_thresh, double sigma, double epsilon, double decision_threshold, int* num_tested_points, double* observed_inlier_ratio)

 Modified version of Wald's SPRT as [Matas]_ et. al. implement it in "Randomized
 RANSAC with Sequential Probability Ratio Test". See the paper for more
 details.

 ``residuals``: Error residuals to use for SPRT analysis.

 ``error_thresh``: Error threshold for determining when Datum fits the model.

 ``sigma``: Probability of rejecting a good model.

 ``epsilon``: Inlier ratio.

 ``decision_threshold``: The decision threshold at which to terminate.

 ``observed_inlier_ratio``: Output parameter of inlier ratio tested.
