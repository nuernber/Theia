.. _chapter-tutorials:

===========================
Documentation and Tutorials
===========================

We have written a set of tutorials to help get you started with using Theia. We have compacted the header files of Theia into modules so that each module represents a certain functionality of the library. There are currently 4 main modules in Theia. When using Theia in your program, only include the module(s) that are needed for your use.

* :ref:`tutorial-ransac` - ``#include <theia/ransac.h>``

* :ref:`tutorial-pose` - ``#include <theia/pose.h>``

* :ref:`tutorial-math` - ``#include <theia/math.h>``

* :ref:`tutorial-point_cloud` - ``#include <theia/point_cloud.h>``

We attempt to provide sufficient documentation in these tutorials but often further documentation can be found in the source code itself. Additionally, (nearly) every file is covered by a unit test that can be viewed as an example use case of the various methods and classes in Theia. If you have looked at the documentation, the source code, and the unit tests and still have confusion please email `the Theia mailing list <http://groups.google.com/group/theia-vision-library>`_ 

Finally, it should be noted that all the code in Theia is under the namespace theia, so you will have to reference that namespace in order to use functions from this library.

.. toctree::
   :maxdepth: 1
   :hidden:

   ransac
   pose
   math
   point_cloud
