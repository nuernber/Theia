.. highlight:: c++

.. default-domain:: cpp

.. _documentation-ransac:

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
				      std::vector<Model>* model) const = 0;
	   virtual double Error(const Datum& data, const Model& model) const = 0;

	   // Functions to optionally implement.
	   virtual bool EstimateModelNonminimal(const std::vector<Datum>& data,
						std::vector<Model>* model) const;
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
interface as a :class:`SampleConsensusEstimator` class. All of the RANSAC
variants in Theia are derived from this class, so they are all guaranteed to
have the same interface. When using a RANSAC (or RANSAC-variant) class, you
simply need to create a ransac object, set up the parameters you want to use,
and then call the :func:`Estimate <SampleConsensusEstimator::Estimate>` method.

.. function:: bool SampleConsensusEstimator::Estimate(const std::vector<Datum>& data, const Estimator<Datum, Model>& estimator, Model* best_model)

  This is the main (and often the only) method you use when performing RANSAC
  (or a variant). It computes a model given the data and the :class:`Estimator`
  class that you have specified for your problem. It returns true (and sets the
  ``best_model`` parameter) upon success, and false (with ``best_model`` having
  undefined behavior) upon failure.

The other main component of using one of the RANSAC methods is to set up the
:class:`RansacParameters` used for the RANSAC scheme. :class:`RansacParameters`
is a struct that holds several crucial elements to deciding how the RANSAC
scheme performs.

.. class:: RansacParameters

  ``error_thresh``: Error threshold to determin inliers for RANSAC (e.g.,
     squared reprojection error). This is what will be used by the estimator to
     determine inliers.

  ``failure_probability``: The failure probability of RANSAC. Set to 0.01 means
        that RANSAC has a 1% chance of missing the correct pose. The default value is
        0.01

  ``min_inlier_ratio``: The minimal assumed inlier ratio, i.e., it is assumed
      that the given set of correspondences has an inlier ratio of at least
      min_inlier_ratio. This is required to limit the number of RANSAC
      iteratios. The default ratio is 0.1

  ``max_iterations``: Another way to specify the maximal number of RANSAC
      iterations. In effect, the maximal number of iterations is set to
      min(max_ransac_iterations, T), where T is the number of iterations
      corresponding to min_inlier_ratio.  This variable is useful if RANSAC is
      to be applied iteratively, i.e., first applying RANSAC with an
      min_inlier_ratio of x, then with one of x-y and so on, and we want to
      avoid repeating RANSAC iterations.  However, the preferable way to limit
      the number of RANSAC iterations is to set min_inlier_ratio and leave
      max_ransac_iterations to its default value.  Per default, this variable is
      set to std::numeric_limits<int>::max().

  ``use_Tdd_test``: Whether to use the T_{d,d}, with d=1, test proposed in [ChumRandomizedRansac]_
      After computing the model, RANSAC selects one match at random and evaluates all
      poses. If the point is an outlier to one pose, the corresponding pose is
      rejected. Notice that if the pose solver returns multiple poses, then at
      most one pose is correct. If the selected match is correct, then only the
      correct pose will pass the test. Per default, the test is disabled.
      NOTE: Not currently implemented!


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
     bool EstimateModel(const std::vector<Point>& data,
                        std::vector<Line>* models) const {
       Line model;
       model.m = (data[1].y - data[0].y)/(data[1].x - data[0].x);
       model.b = data[1].y - model.m*data[1].x;
       models->push_back(model);
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
      int min_num_inliers = 600;
      int max_iters = 1000;

      // Estimate the line with RANSAC.
      LineEstimator line_estimator;
      Line best_line;
      // Create Ransac object, specifying the number of points to sample to
      // generate a model estimation.
      Ransac<Point, Line> ransac_estimator(2);
      // Set the ransac parameters.
      RansacParameters params;
      params.error_thresh = 0.1;
      // Initialize must always be called!
      ransac_estimator.Initialize(params);
      ransac_estimator.Estimate(input_data, line_estimator, &best_line);
      LOG(INFO) << "Line m = " << best_line.m << "*x + " << best_line.b;

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

  .. function:: Ransac(int min_sample_size)

.. class:: Prosac

   Progressive Sampling Consensus as originally proposed by [Chum]_. Input data
   is assumed to have a quality to it, which can then be exposed in your
   sampling strategy by smartly sampling the high quality data points first,
   then progressively sampling the rest of the data set. In the worst case, this
   algorithm degenerates to RANSAC, but typically is significantly faster.

  .. function:: Prosac(int min_sample_size)

  **NOTE:** the :func:`Estimate` method for prosace assumes the data is sorted
    by quality! That is, that the highest quality data point is first, and the
    worst quality data point is last in the input vector.


.. class:: Mlesac

  A generalization of RANSAC that chooses to maximize the likelihood of an estimation rather than the inlier count. Proposed by [Torr]_ et. al.

  .. function:: Mlesac(int min_sample_size)

    ``min_sample_size``: The minimum number of samples needed to estimate a model

.. class:: Arrsac

  Adaptive Real-Time Consensus is a method proposed by [Raguram]_ that utilizes
  pre-emptive techniques to perform a partially depth-first evaluation of many
  generated hypotheses at once. This allows for a bounded running time while
  pursuing only the models which are most likely to lead to high quality
  results. This results in a very fast method which can be used for real-time applications.

  .. function:: Arrsac(int min_sample_size, int max_candidate_hyps = 500, int block_size = 100)

     ``min_sample_size``: The minimum number of samples needed to estimate a model.

     ``max_candidate_hyps``: Maximum number of hypotheses in the initial hypothesis set

     ``block_size``: Number of data points a hypothesis is evaluated against before preemptive ordering is used.


  **NOTE**: This method works for all the unit tests currently in Theia, but needs to be tested further to ensure correctness. Use with caution.


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

    template <class Datum, class Model>
    class Ransac : public SampleConsensusEstimator<Datum, Model> {
     public:
      explicit Ransac(const int min_sample_size)
	  : SampleConsensusEstimator<Datum, Model>(min_sample_size) {}
      virtual ~Ransac() {}

      // Initializes the random sampler and inlier support measurement.
      bool Initialize(const RansacParameters& ransac_params) {
	Sampler<Datum>* random_sampler =
	    new RandomSampler<Datum>(this->min_sample_size_);
	QualityMeasurement* inlier_support =
	    new InlierSupport(ransac_params.error_thresh);
	return SampleConsensusEstimator<Datum, Model>::Initialize(
	    ransac_params, random_sampler, inlier_support);
      }
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
implementation, :class:`Arrsac` does this. See the code for those classes for a
good example on how you should override the :func:`Estimate` method.
