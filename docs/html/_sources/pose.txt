.. highlight:: c++

.. default-domain:: cpp

.. _documentation-pose:

=====================
Pose and Resectioning
=====================

Theia contains efficient and robust implementations of the following pose and
resectioning algorithms. We attempted to make each method as general as possible so that users were not tied to Theia data structures to use the methods. As such, we implemented two interfaces for each pose algorithm: a generic interface that uses primary expressions (e.g., double arrays) and an interface that utilizes the Eigen library vectors and matrices. If a pose algorithm does not have a generic interface, you can request an interface on the `mailing list <http://groups.google.com/group/theia-vision-library>`_

* :ref:`section-p3p`

* :ref:`section-five_point`

* :ref:`section-four_point`

* :ref:`section-eight_point`

* :ref:`section-dls_pnp`

You can include the Pose module in your code with the following line:

.. code-block:: c++

  #include <theia/pose.h>

.. _section-p3p:

Perspective Three Point (P3P)
=============================

  .. function:: int PoseFromThreePoints(const double image_ray[3 * 3], const double points_3d[3 * 3], double solution_rotations[9 * 4], double solution_translations[3 * 4])

  .. function:: bool PoseFromThreePoints(const Eigen::Vector3d image_ray[3], const Eigen::Vector3d world_point[3], std::vector<Eigen::Matrix3d>* solution_rotations, std::vector<Eigen::Vector3d>* solution_translations)

    Computes camera pose using the three point algorithm and returns all possible
    solutions (up to 4). Follows steps from the paper "A Novel Parameterization of
    the Perspective-Three-Point Problem for a direct computation of Absolute
    Camera position and Orientation" by [Kneip]_\. This algorithm has been proven
    to be up to an order of magnitude faster than other methods.

    ``image_ray``: Normalized image rays corresponding to model points.

    ``world_point``: 3D location of features.

    ``solution_rotations``: the rotation matrix of the candidate solutions

    ``solution_translation``: the translation of the candidate solutions

    ``returns``: The number of poses computed, along with the output parameters
    ``rotation`` and ``translation`` filled with the valid poses.

    **NOTE**: P3P returns up to 4 poses, so the rotation and translation arrays are indeed arrays of 3x3 and 3x1 arrays respectively.


  .. function:: int PoseFromThreeCalibrated(const double image_points[2 * 3], const double world_points[3 * 3], const double focal_length[2], const double principal_point[2], double solutions[12 * 4])

     ``image_points``: Location of features on the image plane

     ``world_points``: 3D location of features.

     ``focal_length``: fx, and fy the focal length parameters

     ``principle_point``: the principle point of the image

     ``solutions``: the projection matrices for the candidate solutions

     ``returns``: the number of poses computed.


.. _section-five_point:

Five Point Relative Pose
========================

  .. function:: int FivePointRelativePose(const double image1_points[3 * 5], const double image2_points[3 * 5], double rotation[9 * 10], double translation[3 * 10])

  .. function:: bool FivePointRelativePose(const Eigen::Vector3d image1_points[5], const Eigen::Vector3d image2_points[5], std::vector<Eigen::Matrix3d>* rotation, std::vector<Eigen::Vector3d>* translation)

    Computes the relative pose between two cameras using 5 corresponding
    points. Algorithm is implemented based on "An Efficient Solution to the
    Five-Point Relative Pose Problem" by [Nister]_.

    ``image1_points``: Location of features on the image plane of image 1.

    ``image2_points``: Location of features on the image plane of image 2.

    ``returns``: Output the number of poses computed as well as the relative
    rotation and translation.


.. _section-four_point:

Four Point Algorithm for Homography
===================================

  .. function:: bool FourPointHomography(const std::vector<Eigen::Vector3d>& image_1_points, const std::vector<Eigen::Vector3d>& image_2_points, Eigen::Matrix3d* homography)

  .. function:: bool FourPointHomography(const int num_points, const double image_1_points[], const double image_2_points[], double homography[9])

    Computes the 2D `homography
    <http://en.wikipedia.org/wiki/Homography_(computer_vision)>`_ mapping points
    in image 1 to image 2 such that: :math:`x' = Hx` where :math:`x` is a point in
    image 1 and :math:`x'` is a point in image 2. The algorithm implemented is
    the DLT algorithm based on algorithm 4.2 in [HartleyZisserman]_.

    ``image_1_points``: Image points from image 1. At least 4 points must be
    passed in.

    ``image_2_points``: Image points from image 2. At least 4 points must be
    passed in.

    ``homography``: The computed 3x3 homography matrix.

.. _section-eight_point:

Eight Point Algorithm for Fundamental Matrix
============================================

  .. function:: bool NormalizedEightPoint(const std::vector<Eigen::Vector3d>& image_1_points, const std::vector<Eigen::Vector3d>& image_2_points, Eigen::Matrix3d* fundamental_matrix)

  .. function:: bool NormalizedEightPoint(const int num_points, const double image_1_points[], const double image_2_points[], double fundamental_matrix[9])

    Computes the `fundamental matrix
    <http://en.wikipedia.org/wiki/Fundamental_matrix_(computer_vision)>`_ relating
    image points between two images such that :math:`x' F x = 0` for all
    correspondences :math:`x` and :math:`x'` in images 1 and 2 respectively. The
    normalized eight point algorithm is a speedy estimation of the fundamental
    matrix (Alg 11.1 in [HartleyZisserman]_) that minimizes an algebraic error.

    ``image_1_points``: Image points from image 1. At least 8 points must be
    passed in.

    ``image_2_points``: Image points from image 2. At least 8 points must be
    passed in.

    ``fundamental_matrix``: The computed fundamental matrix.

    ``returns:`` true on success, false on failure.


  .. function:: bool GoldStandardEightPoint(const std::vector<Eigen::Vector3d>& image_1_points, const std::vector<Eigen::Vector3d>& image_2_points, Eigen::Matrix3d* fundamental_matrix)

  .. function:: bool GoldStandardEightPoint(const int num_points, const double image_1_points[], const double image_2_points[], double fundamental_matrix[9])

    Computes the `fundamental matrix
    <http://en.wikipedia.org/wiki/Fundamental_matrix_(computer_vision)>`_
    relating image points between two images such that :math:`x' F x = 0` for
    all correspondences :math:`x` and :math:`x'` in images 1 and 2
    respectively. The gold standard algorithm computes an initial estimation of
    the fundmental matrix from the :func:`NormalizedEightPoint` then uses
    Levenberg-Marquardt to minimize the geometric error (i.e., reprojection
    error) according to algorithm 11.3 in [HartleyZisserman]_.

    ``image_1_points``: Image points from image 1. At least 8 points must be
    passed in.

    ``image_2_points``: Image points from image 2. At least 8 points must be
    passed in.

    ``fundamental_matrix``: The computed fundamental matrix.

    ``returns:`` true on success, false on failure.


.. _section-dls_pnp:

Perspective N-Point
===================

.. function:: void DlsPnp(const std::vector<Eigen::Vector3d>& image_ray, const std::vector<Eigen::Vector3d>& world_point, std::vector<Eigen::Quaterniond>* solution_rotation, std::vector<Eigen::Vector3d>* solution_translation)

  Computes the camera pose using the Perspective N-point method from "A Direct
  Least-Squares (DLS) Method for PnP" by [Hesch]_ and Stergios
  Roumeliotis. This method is extremely scalable and highly accurate for the PnP
  problem. A minimum of 4 points are required, but there is no maximum number of
  points allowed as this is a least-squared approach. Theoretically, up to 27
  solutions may be returned, but in practice only 4 real solutions arise and in
  almost all cases where n >= 6 there is only one solution which places the
  observed points in front of the camera.

  ``image_ray``: Normalized image rays corresponding to model points. Must
  contain at least 4 points.

  ``points_3d``: 3D location of features. Must correspond to the image_ray of
  the same index. Must contain the same number of points as image_ray, and at
  least 4.

  ``solution_rotation``: the rotation quaternion of the candidate solutions

  ``solution_translation``: the translation of the candidate solutions
