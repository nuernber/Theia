SET(DATALOADER_SRCS
  read_bundler_file.cc
  read_bigsfm_binary_file.cc)

# Converts the "raw" bigsfm dataset to a binary file that is much faster to load.
ADD_EXECUTABLE(convert_bigsfm_to_binary convert_bigsfm_to_binary.cc ${DATALOADER_SRCS})
TARGET_LINK_LIBRARIES(convert_bigsfm_to_binary theia ${GFLAGS_LIBRARIES})

# A demo program that will load the bigsfm dataset and render it in OpenGL.
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
ADD_EXECUTABLE(view_bigsfm_dataset view_bigsfm_dataset.cc ${DATALOADER_SRCS})
TARGET_LINK_LIBRARIES(view_bigsfm_dataset theia ${GFLAGS_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARY})
