.. _documentation-sfm:

===========================
Structure from Motion (SfM)
===========================

Theia has implementations of many common Structure from Motion (sfm) algorithms. We
attempt to use a generic interface whenever possible so as to maximize
compatibility with other libraries.

You can include the SfM module in your code with the following line:

.. code-block:: c++

  #include <theia/sfm.h>

Triangulation
=============

Triangulation in structure from motion calculates the 3D position of an image
coordinate that has been tracked through several, if not many, images.

.. function:: Eigen::Vector4d Triangulate(const Matrix3x4d& pose_left, const Matrix3x4d& pose_right, const Eigen::Vector3d& point_left, const Eigen::Vector3d& point_right)

  2-view triangulation using the DLT method described in [HartleyZisserman]_.

.. function:: Eigen::Vector4d TriangulateNView(const std::vector<Matrix3x4d>& poses,
                                 const std::vector<Eigen::Vector3d>& points);

  N-view triangluation that minimizes an algebraic approximation of the
  geometric error.

ICP
===

.. function:: void AlignPointClouds(const double left[][3], const double right[][3], int num_points, double rotation[3][3], double translation[3]);

  We implement ICP for point clouds. We use Besl-McKay registration to align
  point clouds. We use SVD decomposition to find the rotation, as this is much
  more likely to find the global minimum as compared to traditional ICP, which
  is only guaranteed to find a local minimum. Our goal is to find the
  transformation from the left to the right coordinate system. We assume that
  the left and right models have the same number of points, and that the points
  are aligned by correspondence (i.e. left[i] corresponds to right[i]).

  NOTE: SVD is indeed excessive for a 3 dimensional problem, as it leads to SVD
  decomposition of a 3x3. However, after running multiple tests with traditional
  ICP, it was noted that the local minimum achieved was often not
  satisfactory. If runtime becomes an issue, then I may consider switching back
  to an (improved) true ICP method, as it is an analytic and closed form
  solution so it is very fast. Because of this, this is not a true ICP method,
  but it achieves the same registration using a method suggested by Besl-McKay.
