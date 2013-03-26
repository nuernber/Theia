.. highlight:: c++

.. default-domain:: cpp

.. _tutorial-pose:

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


.. function:: int PoseThreePoints(const double image_points[3][3], const double world_points[3][3], double rotation[][3][3], double translation[][3])

  Computes camera pose using the three point algorithm and returns all possible
  solutions (up to 4). Follows steps from the paper "A Novel Parameterization of
  the Perspective-Three-Point Problem for a direct computation of Absolute
  Camera position and Orientation" by [Kneip]_\. This algorithm has been proven
  to be up to an order of magnitude faster than other methods.

  ``image_points``: Location of features on the image plane (x[i][*] = i-th image point).

  ``world_points``: 3D location of features. Must correspond to the image_point of the same index (x[i][*] = i-th world point)

  ``rotation``: The candidate rotations computed from the 3 point algorithm.

  ``translation``: The candidate translations computed.

  ``Returns``: The number of poses computed, along with the output parameters ``rotation`` and ``translation`` filled with the valid poses.

  **NOTE**: P3P returns up to 4 poses, so the rotation and translation arrays are indeed arrays of 3x3 and 3x1 arrays respectively.


.. function:: bool PoseFourPoints(const double image_points[4][3], const double world_points[4][3], double rotation[3][3], double translation[3])

   Computes pose using three point algorithm (method above). The fourth
   correspondence is used to determine the best solution of the (up to 4)
   candidate solutions. All parameter are the same as above, except only the
   best solution is returned in the output parameters, rotation and translation.

   ``Returns``: True if a successful pose is found, false else.


.. _section-five_point:

Five Point Relative Pose
========================

.. function:: std::vector<EssentialMatrix> FivePointRelativePose(const double image1_points[5][3], const double image2_points[5][3])

  Computes the relative pose between two cameras using 5 corresponding
  points. Algorithm is implemented based on "An Efficient Solution to the
  Five-Point Relative Pose Problem" by [Nister]_.

  ``image1_points``: Location of features on the image plane (x[i][*] = i-th image point)

  ``image2_points``: Location of features on the image plane (x[i][*] = i-th image point)

  ``Returns``: Output all solutions of the 5 point algorithm as :class:`EssentialMatrix`.

  **TODO:** Make this output 3x3 double arrays as well.  

