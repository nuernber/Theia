.. _chapter-contributing:

=====================
Contributing to Theia
=====================

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

We use CMake to generate makefiles for Theia to maximize the cross-platform usability. If you need to add a new library or a new file to Theia, you will likely need to add that file to the CMakeLists.txt in src/theia (along with a unit test!).
