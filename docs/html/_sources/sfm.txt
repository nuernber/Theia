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

Projection Matrix
=================

We provide two convenience matrices that are commonly used in multiview geometry. The first is a :class:`TransformationMatrix` which is an affine transformation matrix composed of rotation and translation of the form: :math:`\left[R | t\right]`, i.e., the extrinsic parameters of a camera. The :class:`TransformationMatrix` is merely a typedef of the Eigen affine transformation matrix. Similarly, a :class:`ProjectionMatrix` class is defined that representsthe intrinsic and extrinsic parameters, nameley matrices of the form: :math:`K\left[R | t \right]` where :math:`K` is a 3x3 matrix of the camera intrinsics (e.g., focal length, principle point, and radial distortion). The :class:`ProjectionMatrix` is merely a typedef of an Eigen 3x4 matrix.


2-View Triangulation
====================

Triangulation in structure from motion calculates the 3D position of an image
coordinate that has been tracked through several, if not many, images.

.. function:: Eigen::Vector4d Triangulate(const ProjectionMatrix& pose_left, const ProjectionMatrix& pose_right, const Eigen::Vector3d& point_left, const Eigen::Vector3d& point_right)

  2-view triangulation using the DLT method described in [HartleyZisserman]_.

N-View Triangulation
====================

.. function:: Eigen::Vector4d TriangulateNViewSVD(const std::vector<ProjectionMatrix>& poses,
                                 const std::vector<Eigen::Vector3d>& points);

.. function:: Eigen::Vector4d TriangulateNView(const std::vector<ProjectionMatrix>& poses,
                                 const std::vector<Eigen::Vector3d>& points);

  We provide two N-view triangluation methods that minimizes an algebraic
  approximation of the geometric error. The first is the classic SVD method
  presented in [HartleyZisserman]_. The second is a custom algebraic
  minimization. Note that we can derive an algebraic constraint where we note
  that the unit ray of an image observation can be stretched by depth
  :math:`\alpha` to meet the world point :math:`X` for each of the :math:`n`
  observations:

  .. math:: \alpha_i \bar{x_i} = P_i X,

  for images :math:`i=1,\ldots,n`. This equation can be effectively rewritten as:

  .. math:: \alpha_i = \bar{x_i}^\top P_i X,

  which can be substituted into our original constraint such that:

  .. math:: \bar{x_i} \bar{x_i}^\top P_i X = P_i X
  .. math:: 0 = (P_i - \bar{x_i} \bar{x_i}^\top P_i) X

  We can then stack this constraint for each observation, leading to the linear
  least squares problem:

  .. math:: \begin{bmatrix} (P_1 - \bar{x_1} \bar{x_1}^\top P_1) \\ \vdots \\ (P_n - \bar{x_n} \bar{x_n}^\top P_n) \end{bmatrix} X = \textbf{0}

  This system of equations is of the form :math:`AX=0` which can be solved by
  extracting the right nullspace of :math:`A`. The right nullspace of :math:`A`
  can be extracted efficiently by noting that it is equivalent to the nullspace
  of :math:`A^\top A`, which is a 4x4 matrix.

ICP
===

.. function:: void AlignPointCloudsICP(const int num_points, const double left[], const double right[], double rotation[3 * 3], double translation[3]);

  We implement ICP for point clouds. We use Besl-McKay registration to align
  point clouds. We use SVD decomposition to find the rotation, as this is much
  more likely to find the global minimum as compared to traditional ICP, which
  is only guaranteed to find a local minimum. Our goal is to find the
  transformation from the left to the right coordinate system. We assume that
  the left and right models have the same number of points, and that the points
  are aligned by correspondence (i.e. left[i] corresponds to right[i]).

.. function:: AlignPointCloudsUmeyama(const int num_points, const double left[], const doubel right[], double rotation[3 * 3], double translation[3], double* scale);

  This function estimates the 3D similiarty transformation using the least
  squares method of [Umeyeama]_. The returned rotation, translation, and scale
  align the left points to the right such that :math:`Right = s * R * Left + t`.
