.. _chapter-contributing:

====================
Contribuing to Theia
====================

We welcome and encourage contributions to Theia, whether they are new features,
bug fixes or tests. The `Theia mailing list
<http://groups.google.com/group/theia-vision-library>`_ is the best place for
all development related discussions. Please consider joining it. If you have an
idea for how you'd like to contribute to Theia, please consider emailing the
list first to voice your idea. We can help you fine-tune your idea and this will
also help avoid duplicate work by somebody else who may be working on the same
feature.



Style and Testing
=================

We follow Google's `C++ Style Guide
<http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml>`_ and use git
for version control. We use the GitHub fork and "pull request" feature to add
code to the main branch.

When contributing substantial new code or bug fixes, please add unit tests to
ensure the usage of the code (or to prove the bug is fixed!).


CMake
=====

We have implemented some particular strategies worth noting in our CMake files. We choose to break up the files in Theia into targets. This makes things simpler when compiling -- we only need to include the relevant targets. True, when the library is compiled as a whole all targets will be included anyways, but this strategy reduces the compile time significantly during development and testing.

We have defined a CMake macro to aid in building source targets:

.. function:: CC_LIBRARY(target_name source_file.cc dependency_targets ...)
  
  ``target_name``: Name of the target to be created
  
  ``source_file.cc``: The c++ source file used to generate the target

  ``dependency_targets ...``: A space-delimited list (can be empty) of all targets that this target depends on.

**NOTE** targets are generated according to target_name and the directory where the target resides. For instance, the target "essential_matrix" inside the folder vision/models/ will be referenced as "vision/models/essential_matrix" after it is generated. This also means that dependency_targets should be listed by their full director/target_name style.

We have also implemented a similar macro for testing:

.. function:: GTEST(test_name dependency_targets)

  ``test_name``: The name of the test to be generated. This assumes there is a test file "test_name.cc" in the current folder, and will generate an executable "test_name" in the bin directory.

  ``dependency_targets``: A space-delimited list (can be empty) of all targets that this test depends on.


Since these are non-traditional uses of CMake, we recommend that you check out the CMakeLists.txt files in the source code to get a feel of how to use them before developing on your own.
