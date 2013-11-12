.. highlight:: c++

.. default-domain:: cpp

.. _documentation-features:

========
Features
========

Feature detection and description is a major area of focus in Computer Vision. While SIFT remains the gold standard because of its robustness and matching performance, many other detectors and descriptors are used and often have other competitive advantages. Theia presents friendly classes for feature detection and decription such that the interface is always the same regardless of the methods used.

This module can be included in your code with:

.. code-block:: c++

  #include <theia/features.h>


:class:`Keypoint`
=================

The base :class:`Keypoint` class is a glorified struct that holds information about a keypoint that has been detected with a :class:`KeypointDetector`. Information about the keypoint's position, strength, scale, and orientation can be easily added and retrieved. The type of keypoint can be retrieved with the :func:`keypoint_type()` function.

.. class:: Keypoint

	.. code-block:: c++

           class Keypoint {
	    public:
	      enum KeypointType {
	          INVALID = -1,
		  OTHER = 0,
		  FAST,
		  HARRIS,
		  SIFT,
		  AGAST,
		  BRISK
		  };

	      Keypoint(double x, double y, KeypointType type);
	      ~Keypoint() {}

	      // Required Keypoint type.
	      inline KeypointType keypoint_type() const;
	      inline void set_keypoint_type(KeypointType type);

	      // Required Variable x.
	      inline double x() const;
	      inline void set_x(double x);

	      // Required Variable y.
	      inline double y() const;
	      inline void set_y(double y);

	      // Optional variable strength.
	      inline bool has_strength() const;
	      inline double strength() const;
	      inline void set_strength(double strength);

	      // Optional variable scale.
	      inline bool has_scale() const;
	      inline double scale() const;
	      inline void set_scale(double scale);

	      // Optional variable orientation.
	      inline bool has_orientation() const;
	      inline double orientation() const;
	      inline void set_orientation(double orientation);
	   };

:class:`KeypointDetector`
=========================

Detecting keypoints with Theia is very simple, and we have implemented a number of keypoint detectors that are commonly used in Computer Vision. Each keypoint detector is derived from the virtual class :class:`KeypointDetector`. Each derived class must implement the :func:`DetectKeypoints` method

.. class:: KeypointDetector

  .. function:: bool Initialize()

    This method initializes any internal parameters that must be generated,
    precalculated, or otherwise are independent of the image. The
    :func:`Initialize()` function must be called before using the keypoint
    detector.

  .. function:: bool DetectKeypoints(const GrayImage& input_image, std::vector<Keypoint*>* output_keypoints)

    ``input_image``: The image that you want to detect keypoints on.

    ``ouput_keypoints``: A pointer to a vector that will hold pointers to the
    keypoints detected. Note that the vector should be empty when passed to this
    function. The caller is responsible for deleting the keypoints.

  .. code-block:: c++

    // Assume var keypoint_detector was created with one of the constructors below.

    GrayImage input_image(input_image_filename);
    const bool initialization_success = keypoint_detector.Initialize();

    // Container for the detected keypoints.
    std::vector<Keypoint*> detected_keypoint;
    const bool detection_success =
        keypoint_detector.DetectKeypoints(input_image, &detected_keypoints);


The following keypoint detectors have been implemented in Theia (class constructors are given):

.. function:: FastDetector::FastDetector(int threshold, bool nonmax_suppression, bool strength)

    Set the ``threshold`` for keypoint scores (usually 20 is a good threshold) and
    indicated whether you want to perform nonmaximum suppression. Set ``score`` to
    `true` if you want the corner strength to be set for each keypoint that is
    detected.

.. function:: HarrisDetector::HarrisDetector(int num_corners, double blur, double blur_sigma)

    Set the maximum number of corners to detect. Parameters ``blur`` and
    ``blur_sigma`` specify the amount of blurring to add to the image before
    detecting corners. Default values for ``blur`` and ``blur_sigma`` are 1.0 and
    3.0 respectively.

.. function:: SiftDetector::SiftDetector(int num_octaves, int num_scale_levels, int first_octave)

    The algorithm originally proposed by [Lowe]_ that uses the `VLFeat
    <http://www.vlfeat.org>`_ as the underlying engine.

    Specify the number of image octaves, number of scale levels per octave, and
    where the first octave should start. The default constructor sets these values
    to values -1 (i.e., as many octaves as can be generated), 3, and 0 (i.e., the
    source image)

.. function:: AgastDetector::AgastDetector(AstPattern pattern, int threshold, bool nonmax_suppression)

    The improved FAST detection scheme of [Mair]_ et al.

    ``enum AstPattern`` specifies one of 4 types of sampling patterns for the
    AGAST corner detect: ``AGAST5_8`` is the AGAST pattern with an 8 pixel mask,
    ``AGAST7_12D`` is the AGAST diamond pattern with a 12 pixel mask,
    ``AGAST7_12S`` is the square configuration, and ``OAST9_16`` is the 16 pixel
    mask. By default, we the detector uses ``AGAST5_8`` with a threshold of 30 and
    nonmaximum suppression turn on. More details on the configurations can be
    found at the `AGAST Project website
    <http://www6.in.tum.de/Main/ResearchAgast>`_

.. function:: BriskDetector::BriskDetector(int threshold, int num_octaves)

  The "Binary Robust Invariant Scalable Keypoints" algorithm of [Leutenegger]_
  et al.

  Specify the threshold for keypoint scores (default is 30) and the number of
  octaves to downsample the image (default is 3).

:class:`Descriptor`
===================

.. class:: Descriptor

Theia uses a generic :class:`Descriptor` class as the interface for all descriptor types. This includes floating point and binary descriptors. Like :class:`Keypoints`, descriptors have requried variables ``x``, ``y``, and ``descriptor_type``, as well as optional variables ``strength``, ``scale``, and ``orientation``. All access and "set" methods are the same as for the :class:`Keypoint` class.

Theia has implemented descriptors as one of two subclasses of the :class:`Descriptor` class: :class:`FloatDescriptor` or :class:`BinaryDescriptor`.   All binary descriptors (and extractors) have been implemented very efficiently with SSE and/or ``std::bitset`` instructions. The Theia build system will automatically detect if SSE instructions are available and will use the optimal strategy.

:class:`FloatDescriptor` provides accessors via ``operator []`` to access individual dimensions of the descriptor. :class:`BinaryDescriptor` does not provide such an interface since getting and setting individual bits can be quite messy and dangerous. However, both classes implement accessors to the pointers of the underlying data via calls to :func:`FloatData()` and :func:`CharData()` (the latter can be used with a ``std::bitset`` type to access binary data when the descriptor size is known).


:class:`DescriptorExtractor`
============================

.. class:: DescriptorExtractor

  We enforce a :class:`DescriptorExtractor` interface similar to the
  :class:`KeypointDetector` so that we can extract descriptors at runtime. Each
  descriptor has a corresponding extractor class that is used to compute that
  descriptor given keypoints. However, we must call the :func:`Initialize()`
  method before computing descriptors.

  .. function:: bool Initialize()

    This method initializes any internal parameters that must be generated,
    precalculated, or otherwise are independent of the image. The
    :func:`Initialize()` function must be called before using the keypoint
    detector.

  .. function:: Descriptor* ComputeDescriptor(const GrayImage& input_image, const Keypoint& keypoints)

    ``input_image``: The image that you want to detect keypoints on.

    ``keypoint``: The keypoint that the descriptor will be computed from.

    ``returns Descriptor*``: returns a :class:`Descriptor` that has been created
    from the keypoint passed to the function. Caller is responsible for deleting
    the descriptor.

  .. function:: bool ComputeDescriptors(const GrayImage& input_image, const std::vector<Keypoint*>& keypoints, std::vector<Descriptor*>* output_descriptors)

    ``input_image``: The image that you want to detect keypoints on.

    ``keypoints``: A vector of the keypoint pointers that will have descriptors
    extracted.

    ``ouput_descriptors``: A pointer to a vector that will hold pointers to the
    descriptors computed. Note that the vector should be empty when passed to this
    function. The caller is responsible for deleting the keypoints.

  .. code-block:: c++

    // Open image we want to extract features from.
    GrayImage input_image(input_image_filename);

    // Detect keypoints.
    SiftDetector sift_keypoint_detector;
    const bool keypoint_init_success = sift_keypoint_detector.Initialize();
    std::vector<Keypoint> sift_keypoints;
    const bool detection_success =
        sift_keypoint_detector.DetectKeypoints(input_image, &sift_keypoints);

    // Initialize descriptor extractor.
    SiftDescriptorExtractor sift_extractor;
    const bool descriptor_init_succes = sift_extractor.Initialize();

    // E.g., compute a single descriptor
    Descriptor* sift_descriptor =
      sift_extractor.ComputeDescriptor(input_image, *keypoint[0]);

    // E.g., compute many descriptors.
    std::vector<Descriptor*> sift_descriptors;
    const bool extraction_success =
      sift_extractor.ComputeDescriptors(image, sift_keypoints, &sift_descriptors)

We implement the following descriptor extractors (and corresponding descriptors) in Theia (constructors are given).

.. function:: PatchDescriptorExtractor::PatchDescriptorExtractor(int patch_rows, int patch_cols)

  Specify the size of the patch to extract from the image.

.. function:: SiftDescriptorExtractor::SiftDescriptorExtractor(int num_octaves, int num_scale_levels, int first_octave)

  The algorithm originally proposed by [Lowe]_ that uses the `VLFeat
  <http://www.vlfeat.org>`_ as the underlying engine.

  We only implement the standard 128-dimension descriptor. Specify the number
  of image octaves, number of scale levels per octave, and where the first
  octave should start. The default constructor sets these values to values -1
  (i.e., as many octaves as can be generated), 3, and 0 (i.e., the source
  image). Typically these parameters are set to match the :class:`SiftDetector`
  parameters.

.. function:: FreakDescriptorExtractor::FreakDescriptorExtractor(bool rotation_invariant, bool scale_invariant, int num_octaves)

  The "Fast Retina Keypoint" algorithm proposed by [Alahi]_ et al.

  ``rotation_invariant``: Set to true if you want to normalize the orientation of the keypoints before computing the descriptor.

  ``scale_invariant``: Set to true if you want to normalize the scale of keypoints before computing the descriptor.

  ``num_octaves``: The number of octaves that the keypoints span.

  The :class:`FreakDescriptorExtractor` is typically used with the
  :class:`BriskDetector` to detect keypoints.

.. function:: BriskDescriptorExtractor::BriskDescriptorExtractor(bool rotation_invariant, bool scale_invariant, float pattern_scale)

  The "Binary Robust Invariant Scalable Keypoints" algorithm of [Leutenegger]_
  et al.

  ``rotation_invariant``: Set to true if you want to normalize the orientation of the keypoints before computing the descriptor.

  ``scale_invariant``: Set to true if you want to normalize the scale of keypoints before computing the descriptor.

  ``pattern_scale``: Scale of the BRISK pattern to use.
