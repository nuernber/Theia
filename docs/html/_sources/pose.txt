.. highlight:: c++

.. default-domain:: cpp

.. _documentation-pose:

=====================
Pose and Resectioning
=====================

Theia contains efficient and robust implementations of the following pose and
resectioning algorithms. We attempted to make each method as general as possible so that users were not tied to Theia data structures to use the methods. This is why most methods use simple arrays as the input and output -- we give the user the power to write simple wrappers for their own data structures if they so choose.

* :ref:`section-p3p`

* :ref:`section-five_point`


You can include the Pose module in your code with the following line:

.. code-block:: c++

  #include <theia/pose.h>

.. _section-p3p:

Perspective Three Point (P3P)
=============================


.. function:: int PoseFromThreePoints(const double image_ray[3 * 3], const double points_3d[3 * 3], double solution_rotations[9 * 4], double solution_translations[3 * 4])

  Computes camera pose using the three point algorithm and returns all possible
  solutions (up to 4). Follows steps from the paper "A Novel Parameterization of
  the Perspective-Three-Point Problem for a direct computation of Absolute
  Camera position and Orientation" by [Kneip]_\. This algorithm has been proven
  to be up to an order of magnitude faster than other methods.

  ``image_ray``: Normalized image rays corresponding to model points.

  ``world_point``: 3D location of features. Must correspond to the image_ray of
     the same index (x[i][*] = i-th world point)

  ``solution_rotations``: the rotation matrix of the candidate solutions

  ``solution_translation``: the translation of the candidate solutions

  ``Returns``: The number of poses computed, along with the output parameters ``rotation`` and ``translation`` filled with the valid poses.

  **NOTE**: P3P returns up to 4 poses, so the rotation and translation arrays are indeed arrays of 3x3 and 3x1 arrays respectively.


.. function:: int PoseFromThreeCalibrated(const double image_points[2 * 3], const double world_points[3 * 3], const double focal_length[2], const double principal_point[2], double solutions[12 * 4])

   ``image_points``: Location of features on the image plane

   ``world_points``: 3D location of features. Must correspond to the image_point
     of the same index (x[i][*] = i-th world point)

   ``focal_length``: fx, and fy the focal length parameters

   ``principle_point``: the principle point of the image

   ``solutions``: the projection matrices for the candidate solutions

   ``Return``: the number of poses computed.



.. _section-five_point:

Five Point Relative Pose
========================

.. function:: int FivePointRelativePose(const double image1_points[3 * 5], const double image2_points[3 * 5], double rotation[9 * 10], double translation[3 * 10])


  Computes the relative pose between two cameras using 5 corresponding
  points. Algorithm is implemented based on "An Efficient Solution to the
  Five-Point Relative Pose Problem" by [Nister]_.

  ``image1_points``: Location of features on the image plane of image 1.

  ``image2_points``: Location of features on the image plane of image 2.

  ``Returns``: Output the number of poses computed as well as the relative
  rotation and translation.
