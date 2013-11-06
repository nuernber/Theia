CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
IF (COMMAND cmake_policy)
  CMAKE_POLICY(SET CMP0003 NEW)
ENDIF (COMMAND cmake_policy)

PROJECT(THEIA C CXX)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

# ==============================================================================
# Additional cmake find modules
# ==============================================================================
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmakeFindModules)
include(FindTheiaDependencies)

# Add GTest if testing is enabled
OPTION(BUILD_TESTING
  "Enable testing"
  ON)
ENABLE_TESTING()
ADD_DEFINITIONS(-DGTEST_USE_OWN_TR1_TUPLE=1)

set_compile_parameters()
find_theia_dependencies()
include(theia_macros)

include (OptimizeForArchitecture)
OptimizeForArchitecture()

IF (SSE2_FOUND )
  IF(CMAKE_BUILD_TYPE STREQUAL "Release")
    ADD_DEFINITIONS(-DTHEIA_USE_SSE)
  ENDIF (CMAKE_BUILD_TYPE STREQUAL "Release")
ENDIF (SSE2_FOUND)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)


# build Third party libraries
ADD_SUBDIRECTORY(libraries)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/libraries
  ${flann_SOURCE_DIR}/src/cpp
  ${gtest_SOURCE_DIR}/include
  ${gtest_SOURCE_DIR}
  ${hayai_SOURCE_DIR}
  ${vlfeat_SOURCE_DIR}
  ${agast_SOURCE_DIR})

# Include all source directories.
ADD_SUBDIRECTORY(image)
ADD_SUBDIRECTORY(math)
ADD_SUBDIRECTORY(solvers)
ADD_SUBDIRECTORY(tutorials)
ADD_SUBDIRECTORY(util)
ADD_SUBDIRECTORY(vision)

# ---------------- Install commands ----------------- #
# Compile protos first if desired.
IF(${PROTOBUF_FOUND})
  file(GLOB_RECURSE ProtoFiles "${CMAKE_SOURCE_DIR}/*.proto")
  THEIA_PROTOBUF_GENERATE_CPP(ProtoSource ProtoHeader ${ProtoFiles})
ENDIF(${PROTOBUF_FOUND})

# Install the .h files
install(DIRECTORY ${CMAKE_SOURCE_DIR} DESTINATION include
  FILES_MATCHING REGEX PATTERN "*.h"
  PATTERN "data*" EXCLUDE
  PATTERN "docs*" EXCLUDE
  PATTERN "build*" EXCLUDE
  PATTERN "libraries*" EXCLUDE
  PATTERN "*git*" EXCLUDE)

# Install the library
LIST(REMOVE_DUPLICATES THEIA_SRC_DEPS)
ADD_LIBRARY(theia STATIC ${THEIA_SRC_FILES})
TARGET_LINK_LIBRARIES(theia ${THEIA_LIBRARY_DEPENDENCIES} ${THEIA_SRC_DEPS})
INSTALL(TARGETS theia DESTINATION lib)
