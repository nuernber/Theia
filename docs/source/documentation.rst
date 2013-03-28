.. _chapter-documentation:

=============
Documentation
=============

We have compacted Theia into modules so that each module represents a certain functionality of the library. There are currently 5 main modules in Theia, the contents of which can be added to your program with a single include line for each module. When using Theia in your program, only include the module(s) that are needed for your use.

* :ref:`documentation-image` - ``#include <theia/image.h>``

* :ref:`documentation-ransac` - ``#include <theia/ransac.h>``

* :ref:`documentation-pose` - ``#include <theia/pose.h>``

* :ref:`documentation-math` - ``#include <theia/math.h>``

* :ref:`documentation-point_cloud` - ``#include <theia/point_cloud.h>``

We attempt to provide sufficient documentation but often further documentation can be found in the source code itself. You will likely find the :ref:`chapter-tutorials` useful as well. Additionally, (nearly) every file is covered by a unit test that can be viewed as an example use case of the various methods and classes in Theia. If you have looked at the documentation, the tutorials, the source code, and the unit tests and still have confusion please email `the Theia mailing list <http://groups.google.com/group/theia-vision-library>`_ 

Finally, it should be noted that all the code in Theia is under the namespace theia, so you will have to reference that namespace in order to use functions from this library.

.. toctree::
   :maxdepth: 1
   :hidden:
   
   image
   ransac
   pose
   math
   point_cloud
