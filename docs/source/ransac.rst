.. highlight:: c++

.. default-domain:: cpp

.. _tutorial-ransac:

======
Ransac
======

`Random Sample Consensus <http://en.wikipedia.org/wiki/RANSAC>`_, or RANSAC, one
of the most commonly used algorithms in Computer Vision. As a result, much
research has gone into making RANSAC extensions and variants that increase the
efficiency or accuracy of the estimation. We have implemented a templated class
that makes using RANSAC for estimation extremely easy as well as simple to
extend.

This module can be included in your code with:

.. code-block:: c++
 
  #include <theia/ransac.h>

**NOTE**: For the descriptions below, we often use the term "RANSAC" to mean the general strategy of model estimation via sample consensus. Most of the time, "RANSAC" refers to RANSAC and the variants we have implemented.

The following RANSAC methods are implemented in Theia:

* :class:`Ransac`
* :class:`Prosac`
* :class:`Mlesac`
* :class:`Arrsac`
* :class:`Recon`

:class:`Estimator`
==================

The basic method for using RANSAC (and its variants) is to specify the class
corresponding to the algorithm you will use (e.g. RANSAC, PROSAC, etc.) and the
method for estimating a model from data points. The interface to do the latter
requires you implement derived class of the :class:`Estimator` class.

.. class:: Estimator
	
	.. code-block:: c++
	
	 template <class Datum, class Model>
	 class Estimator {
	  public:
	   Estimator() {}
	   virtual ~Estimator() {}
	   virtual bool EstimateModel(const std::vector<Datum>& data,
				      Model* model) const = 0;
	   virtual double Error(const Datum& data, const Model& model) const = 0;

	   // Functions to optionally implement.
	   virtual bool EstimateModelNonminimal(const std::vector<Datum>& data,
						Model* model) const;
	   virtual bool RefineModel(const std::vector<Datum>& data, Model* model) const;
	   virtual bool ValidModel(const Model& model) const;

	   // Helper methods implemented in base class.
	   virtual std::vector<double> Residuals(const std::vector<Datum>& data,
						 const Model& model) const;

	   std::vector<bool> GetInliers(const std::vector<Datum>& data,
					const Model& model,
					double error_threshold) const;

	   int GetNumInliers(const std::vector<Datum>& data,
			     const Model& model,
			     double error_threshold) const;
	 };

	The only methods that are required to be implemented are the
	:func:`Estimator::EstimateModel` and :func:`Estimator::Error`
	methods. These methods specify how the model is estimated from the data
	provided, and how the error residuals are calculated from a given
	model. All other methods are optional to implement, but will only
	enhance the output of RANSAC.

Using the RANSAC classes
========================

In order to make our RANSAC classes consistent and extendible we specify an
interface (via a pure virtual class) as a :class:`SampleConsensusEstimator`
class. All of the RANSAC variants in Theia are derived from this class, so they
are all guaranteed to have the same interface. When using a RANSAC (or
RANSAC-variant) class, you simply need to call the constructor (each class
implements its own :ref:`constructor <section-constructors>`) and then call the
:func:`Estimate <SampleConsensusEstimator::Estimate>` method.

.. function:: bool SampleConsensusEstimator::Estimate(const std::vector<Datum>& data, const Estimator<Datum, Model>& estimator, Model* best_model)

  This is the main (and often the only) method you use when performing RANSAC
  (or a variant). It computes a model given the data and the :class:`Estimator`
  class that you have specified for your problem. It returns true (and sets the
  ``best_model`` parameter) upon success, and false (with ``best_model`` having
  undefined behavior) upon failure.

We will illustrate the use of the RANSAC class with a simple line estimation example.

  .. code-block:: c++

   // Our "data".
   struct Point {
     double x; double y;
   };

   // Our "model".
   struct Line {
     double m; double b;
   };

   // Estimator class.
   class LineEstimator: public Estimator<Point, Line> {
     // Estimate a line from two points.
     bool EstimateModel(const std::vector<Point>& data, Line* model) const {
       model->m = (data[1].y - data[0].y)/(data[1].x - data[0].x);
       model->b = data[1].y - model->m*data[1].x;
       return true;
     }
     
     // Calculate the error as the y distance of the point to the line.
     double Error(const Point& point, const Line& line) const {
       return point.y - (line.m*point.x + line.b);
     }
   };

Specifying an :class:`Estimator` is that easy! Now lets look at how to actually
use a RANSAC method to use the :class:`LineEstimator`.

  .. code-block:: c++

    int main (int argc, char** argv) {
      // Generate your input data using your desired method.
      // We put pseudo-code here for simplicity.
      std::vector<Point> input_data;

      // Add 700 inliers.
      for (int i = 0; i < 700; i++) {
        input_data.push_back(inlier_point);
      }
      // Add 300 outliers.
      for (int i = 0; i < 300; i++) {
        input_data.push_back(outlier_point);
      }	

      // Specify RANSAC parameters.
      double error_threshold = 0.3;
      int min_num_inliers = 700;
      int max_iters = 10000;

      // Estimate the line with RANSAC.
      LineEstimator line_estimator;
      Line best_line;
      Ransac<Point, Line> ransac_estimator(2, error_threshold, min_num_inliers, max_iters);
      ransac_estimator.Estimate(input_data, line_estimator, &best_line);
      std::cout << "Line m = " << best_line.m << "*x + " << best_line.b << std::endl;

      return 0;
    }

There you have it. With just a few lines of code we can use RANSAC to estimate
the best fitting line. You could easily swap the :class:`Ransac` class with any
of the RANSAC variants implemented in Theia without having to change anything
else in the code.

.. _section-constructors:

Instances of RANSAC Methods
===========================

Theia has implemented several RANSAC methods as derived classes of the
:class:`SampleConsensusEstimator` class. The typical use case is still to call
the :func:`Estimate` method, but each method is likely to have a different
constructor. The constructors for each method are specified as follows

.. class:: Ransac

  The standard `RANSAC <http://en.wikipedia.org/wiki/RANSAC>`_ implementation as originally proposed by Fischler et. al. [Fischler]_

  .. function:: Ransac(int min_sample_size, double error_threshold, int min_num_inliers, int max_iters)

    ``min_sample_size``: The minimum number of samples needed to estimate a model

    ``error_threshold``: Error threshold for determining if a data point is an inlier or not.

    ``min_num_inliers``: Minimum number of inliers needed to terminate.

    ``max_iters``: Maximum number of iterations to run RANSAC. To set the number of iterations based on the outlier probability, use SetMaxIters.

  Alternatively, you can choose to have the algorithm calculate the maximum number of iterations by specifying the outlier probability and the probability of having an uncontaminated model according to eq 4.18 in [HartleyZisserman]_

  .. function:: Ransac(int min_sample_size, double error_threshold, int min_num_inliers, double outlier_probability, double no_fail_probability = 0.99)

    ``min_sample_size``: The minimum number of samples needed to estimate a model

    ``error_threshold``: Error threshold for determining if a data point is an inlier or not.

    ``min_num_inliers``: Minimum number of inliers needed to terminate.

    ``outlier_probability``: Probabiliy that a given data point is an outlier.

    ``no_fail_probability``: Probability that at least one sample has no outliers.

.. class:: Prosac

   Progressive Sampling Consensus as originally proposed by [Chum]_. Input data
   is assumed to have a quality to it, which can then be exposed in your
   sampling strategy by smartly sampling the high quality data points first,
   then progressively sampling the rest of the data set. In the worst case, this
   algorithm degenerates to RANSAC, but typically is significantly faster.

  .. function:: Prosac(int min_sample_size, double error_threshold, int min_num_inliers, int max_iters)

    ``min_sample_size``: The minimum number of samples needed to estimate a model

    ``error_threshold``: Error threshold for determining if a data point is an inlier or not.

    ``min_num_inliers``: Minimum number of inliers needed to terminate.

    ``max_iters``: Maximum number of iterations to run PROSAC. To set the number of iterations based on the outlier probability, use SetMaxIters.

  **NOTE:** the :func:`Estimate` method for prosace assumes the data is sorted by quality! That is, that the highest quality data point is first, and the worst quality data point is last in the input vector.


.. class:: Mlesac

  A generalization of RANSAC that chooses to maximize the likelihood of an estimation rather than the inlier count. Proposed by [Torr]_ et. al.

  .. function:: Mlesac(int min_sample_size, double inlier_mean, double inlier_sigma, double search_left, double search_right, const std::vector<double>& confidence, double confidence_threshold)

    ``min_sample_size``: The minimum number of samples needed to estimate a model

    ``inline_mean``: Mean of inlier noise distribution.

    ``inlier_sigma``: Sigma of the inlier noise distribution.

    ``search_left``: Left bound of the search region. e.g. -100px for image correspondences

    ``search_right``: Right bound of the search region. e.g. 100px for image correspondences

    ``confidence``: Vector containing the confidences of each data point.

    ``confidence_threshold``: Correspondances above this are considered inliers.


.. class:: Arrsac

  Adaptive Real-Time Consensus is a method proposed by [Raguram]_ that utilizes
  pre-emptive techniques to perform a partially depth-first evaluation of many
  generated hypotheses at once. This allows for a bounded running time while
  pursuing only the models which are most likely to lead to high quality
  results. This results in a very fast method which can be used for real-time applications.

  .. function:: Arrsac(int min_sample_size, double error_thresh, int max_candidate_hyps = 500, int block_size = 100)

     ``min_sample_size``: The minimum number of samples needed to estimate a model.

     ``error_thresh``: Error threshold for determining inliers vs. outliers. i.e. if the error is below this, the data point is an inlier.

     ``max_candidate_hyps``: Maximum number of hypotheses in the initial hypothesis set

     ``block_size``: Number of data points a hypothesis is evaluated against before preemptive ordering is used.


  **NOTE**: This method works for all the unit tests currently in Theia, but needs to be tested further to ensure correctness. Use with caution.

.. class:: Recon

  Residual Consensus estimation as proposed by [RaguramFrahm]_. The driving idea
  is to generate several models from random samples, then compare the
  distribution of their residuals. If the same data points all have small
  residuals, then these points are likely to be inliers. If enough models agree
  with each other on this principle, then the models are likely to be drawn from
  inliers (i.e. uncontaminated models). For more details, refer to the paper.

  .. function:: Recon(int min_sample_size, int min_consisten_models, double sigma_max)

    ``min_sample_size``: The minimum number of samples needed to estimate a model

    ``min_consistent_models``: Number of consistent models that must be generated before a solution is determined.

    ``sigma_max``: A *rough* estimate of the maximum noise variance of the inlier poitns. This only needs to be correct up to an order of magnitude in order to be useful.

  **NOTE**: Our implementation was not able to achieve the stability that the paper records. We had great difficulty in distinguishing all-outliers cases with all-inliers cases using the KS test as recommended. Thus, the current implementation requires a maximum estimation of sigma (the inlier noise) to be provided.


Implementing a New RANSAC Method
================================

The :class:`SampleConsensusEstimator` class consists of two main items: a
:class:`Sampler` and a :class:`QualityMeasurement`. These two members specify
the most important aspects of most RANSAC techniques: how the data is sampled
(:class:`Sampler`) and how the model quality (or, conversely, error) is measured
(:class:`QualityMeasurement`). Adjusting the :class:`Sampler` is how techniques
such as PROSAC achieve success. Adjusting the measurement of model quality from
the trivial method (e.g. counting inliers) is how methods such as MLESAC achieve
good results. Both the :class:`Sampler` and :class:`QualityMeasurement` classes
are pure virtual classes that must be derived for all RANSAC methods. Further,
the :func:`Estimate` method implemented in the :class:`SampleConsensusEstimator`
base class performs a typical RANSAC style routine, sampling according to the
:class:`Sampler` and :class:`QualityMeasurement` specified. 

To implement a new RANSAC method, you should create a class derived from
:class:`SampleConsensusEstimator`. Most methods will probably involve simply
using a new sampler or quality measurement class, as the :func:`Estimate`
function will not change and can simply be inherited from the
:class:`SampleConsensus` class. In those cases, you can follow the model of the
:class:`Ransac` class to specify your new RANSAC-variant class:

  .. code-block:: c++

    template<class Datum, class Model>
    class Ransac : public SampleConsensusEstimator<Datum, Model> {
     public:
  
      Ransac(int min_sample_size,
             double error_threshold,
       	     int min_num_inliers,
             int max_iters)
        : SampleConsensusEstimator<Datum, Model>(
            new RandomSampler<Datum>(min_sample_size),
            new InlierSupport(error_threshold,
                              min_num_inliers),
            max_iters) {}
    };

This is all that the :class:`Ransac` class needs to specify, and the
:func:`Estimate` function implemented in the base class
(:class:`SampleConsensusEstimator`) will use the :class:`RandomSampler` to
randomly sample the data, and :class:`InlierSupport` to calculate inliers. Of
course, :class:`RandomSampler` and :class:`InliersSupport` are derived classes
of :class:`Sampler` and :class:`QualityMeasurement` respectively. See the code
for more details.

If you want to create a new RANSAC method that involves changing the way
estimation happens, your class can override the :func:`Estimate` method. For our
implementation, :class:`Recon` and :class:`Arrsac` both do this. See the code
for those classes for a good example on how you should override the
:func:`Estimate` method.

