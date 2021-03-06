# Copyright (C) 2013 The Regents of the University of California (Regents).
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
#
#     * Neither the name of The Regents or University of California nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Please contact the author of this library if you have any questions.
# Author: Chris Sweeney (cmsweeney@cs.ucsb.edu)

# Add headers
FILE(GLOB THEIA_HDRS *.h)

# Add sources
SET(THEIA_SRC
  data_loader/read_bigsfm_binary_file.cc
  data_loader/read_bundler_file.cc
  image/descriptor/brisk_descriptor.cc
  image/descriptor/descriptor_extractor.cc
  image/descriptor/freak_descriptor.cc
  image/descriptor/patch_descriptor.cc
  image/descriptor/sift_descriptor.cc
  image/image_canvas.cc
  image/keypoint_detector/agast_detector.cc
  image/keypoint_detector/brisk_detector.cc
  image/keypoint_detector/brisk_impl.cc
  image/keypoint_detector/fast_detector.cc
  image/keypoint_detector/harris_detector.cc
  image/keypoint_detector/sift_detector.cc
  math/closed_form_polynomial_solver.cc
  math/polynomial.cc
  math/probability/sequential_probability_ratio.cc
  util/random.cc
  util/stringprintf.cc
  vision/sfm/camera/camera_pose.cc
  vision/sfm/pose/dls_impl.cc
  vision/sfm/pose/dls_pnp.cc
  vision/sfm/pose/eight_point_fundamental_matrix.cc
  vision/sfm/pose/five_point_focal_length_radial_distortion.cc
  vision/sfm/pose/five_point_relative_pose.cc
  vision/sfm/pose/four_point_focal_length.cc
  vision/sfm/pose/four_point_focal_length_helper.cc
  vision/sfm/pose/four_point_homography.cc
  vision/sfm/pose/perspective_three_point.cc
  vision/sfm/pose/util.cc
  vision/sfm/projection_matrix.cc
  vision/sfm/triangulation/triangulation.cc
  vision/transformation/align_point_clouds.cc
  )

SET(THEIA_LIBRARY_DEPENDENCIES
  ${CERES_LIBRARIES}
  ${CVD_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  vlfeat
  agast)

IF (OPENMP_FOUND)
  IF (NOT MSVC)
    LIST(APPEND THEIA_LIBRARY_DEPENDENCIES gomp)
  ENDIF (NOT MSVC)
ENDIF (OPENMP_FOUND)

SET(THEIA_LIBRARY_SOURCE
  ${THEIA_SRC}
  ${THEIA_HDRS})

ADD_LIBRARY(theia ${THEIA_LIBRARY_SOURCE})
SET_TARGET_PROPERTIES(theia PROPERTIES
  VERSION ${THEIA_VERSION}
  SOVERSION ${THEIA_VERSION_MAJOR}
  )
TARGET_LINK_LIBRARIES(theia ${THEIA_LIBRARY_DEPENDENCIES})

INSTALL(TARGETS theia
  EXPORT  TheiaExport
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib${LIB_SUFFIX}
  ARCHIVE DESTINATION lib${LIB_SUFFIX})

IF (BUILD_TESTING)
  ADD_DEFINITIONS(-DTHEIA_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/data")
  MACRO (GTEST FILENAME)
    STRING(REPLACE "/" ";" TEST_NAME_LIST ${FILENAME})
    LIST(REVERSE TEST_NAME_LIST)
    LIST(GET TEST_NAME_LIST 0 TEST_NAME)
    ADD_EXECUTABLE(${TEST_NAME}_test test/test_main.cc ${FILENAME}_test.cc)
    TARGET_LINK_LIBRARIES(${TEST_NAME}_test
      gtest
      theia
      ${THEIA_LIBRARY_DEPENDENCIES})
    ADD_TEST(NAME ${TEST_NAME}_test
      COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TEST_NAME}_test)
  ENDMACRO (GTEST)

  GTEST(image/descriptor/brisk_descriptor)
  GTEST(image/descriptor/freak_descriptor)
  GTEST(image/descriptor/patch_descriptor)
  GTEST(image/descriptor/sift_descriptor)
  GTEST(image/image)
  GTEST(image/keypoint_detector/agast_detector)
  GTEST(image/keypoint_detector/brisk_detector)
  GTEST(image/keypoint_detector/fast_detector)
  GTEST(image/keypoint_detector/harris_detector)
  GTEST(image/keypoint_detector/sift_detector)
  GTEST(math/closed_form_polynomial_solver)
  GTEST(math/matrix/gauss_jordan)
  GTEST(math/polynomial)
  GTEST(math/probability/sprt)
  GTEST(solvers/arrsac)
  GTEST(solvers/mlesac)
  GTEST(solvers/prosac)
  GTEST(solvers/ransac)
  GTEST(vision/matching/distance)
  GTEST(vision/sfm/camera/camera_pose)
  GTEST(vision/sfm/pose/dls_pnp)
  GTEST(vision/sfm/pose/eight_point_fundamental_matrix)
  GTEST(vision/sfm/pose/five_point_focal_length_radial_distortion)
  GTEST(vision/sfm/pose/five_point_relative_pose)
  GTEST(vision/sfm/pose/four_point_focal_length)
  GTEST(vision/sfm/pose/four_point_homography)
  GTEST(vision/sfm/pose/perspective_three_point)
  GTEST(vision/sfm/triangulation/triangulation)
  GTEST(vision/transformation/align_point_clouds)

ENDIF (BUILD_TESTING)