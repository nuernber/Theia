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

For polynomials of degree > 4 there are no easy closed-form solutions, making the problem of finding roots much more difficult. However, we have implemented a function that will solve for all roots of a polynomial by constructing a companion matrix and solving with an eigenvalue decomposition.

.. function:: std::vector<std::complex<double> > GetPolynomialRoots(const Eigen::VectorXd& coeffs)
.. function:: std::vector<double> GetRealPolynomialRoots(const Eigen::VectorXd& coeffs)

  These methods take in polynomial coefficients such that the highest degree
  coefficient is the first element, and the lowest degree coefficient
  (correspondening to degree 0) is the last element. Real or complex roots are
  returned depending on the method called.

.. _section-gauss_jordan:

Guass-Jordan
============



.. function:: void GaussJordan(Eigen::MatrixBase<Derived>* input, int max_rows = 99999)

  Perform traditional Gauss-Jordan elimination on an Eigen3 matrix. If
  ``max_rows`` is specified, it will on perform Gauss-Jordan on the first
  ``max_rows`` number of rows. This is useful for problems where your system is
  extremely overdetermined and you do not need all rows to be solved.


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
