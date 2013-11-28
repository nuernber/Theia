# Add headers
FILE(GLOB THEIA_HDRS *.h)

# Add sources
SET(THEIA_SRC
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
  math/probability/sequential_probability_ratio.cc
  util/random.cc
  vision/pose/dls_impl.cc
  vision/pose/dls_pnp.cc
  vision/pose/five_point_relative_pose.cc
  vision/pose/perspective_three_point.cc
  vision/pose/util.cc
  vision/transformation/align_point_clouds.cc
  vision/triangulation/triangulation.cc
  )

SET(THEIA_LIBRARY_DEPENDENCIES
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${CVD_LIBRARIES}
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
  GTEST(vision/pose/dls_pnp)
  GTEST(vision/pose/five_point_relative_pose)
  GTEST(vision/pose/perspective_three_point)
  GTEST(vision/transformation/align_point_clouds)
  GTEST(vision/triangulation/triangulation)

ENDIF (BUILD_TESTING)