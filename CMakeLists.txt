CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
IF (COMMAND cmake_policy)
  CMAKE_POLICY(SET CMP0003 NEW)
ENDIF (COMMAND cmake_policy)

PROJECT(THEIA C CXX)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

# ==============================================================================
# Additional cmake find modules
# ==============================================================================
SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmakeFindModules")

OPTION(BUILD_TESTING "Enable testing" ON)
OPTION(OPENMP "Enable thread solving (requires OpenMP)" ON)
OPTION(BUILD_TUTORIALS "Build tutorial programs" ON)
OPTION(BUILD_DOCUMENTATION "Build html User's Guide" OFF)

ENABLE_TESTING()
ADD_DEFINITIONS(-DGTEST_USE_OWN_TR1_TUPLE=1)
SET(GTEST_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/libraries/gtest")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

SET(THEIA_VERSION_MAJOR 0)
SET(THEIA_VERSION_MINOR 1)
SET(THEIA_VERSION_PATCH 0)
SET(THEIA_VERSION
    ${THEIA_VERSION_MAJOR}.${THEIA_VERSION_MINOR}.${THEIA_VERSION_PATCH})
SET(THEIA_ABI_VERSION 0.1.0)

#Include source subdirectory.
INCLUDE("${CMAKE_MODULE_PATH}/FindSSE.cmake")
FindSSE()
IF (SSE2_FOUND)
  IF(CMAKE_BUILD_TYPE STREQUAL "Release")
    ADD_DEFINITIONS(-DTHEIA_USE_SSE)
  ENDIF (CMAKE_BUILD_TYPE STREQUAL "Release")
ENDIF (SSE2_FOUND)

# Eigen
FIND_PACKAGE(Eigen REQUIRED)
IF (EIGEN_FOUND)
  MESSAGE("-- Found Eigen version ${EIGEN_VERSION}: ${EIGEN_INCLUDE_DIRS}")
ENDIF (EIGEN_FOUND)

# Use a larger inlining threshold for Clang, since it hobbles Eigen,
# resulting in an unreasonably slow version of the blas routines. The
# -Qunused-arguments is needed because CMake passes the inline
# threshold to the linker and clang complains about it and dies.
IF (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  SET(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Qunused-arguments -mllvm -inline-threshold=600")
  # Older versions of Clang (<= 2.9) do not support the 'return-type-c-linkage'
  # option, so check for its presence before adding it to the default flags set.
  INCLUDE(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-Wno-return-type-c-linkage"
    HAVE_RETURN_TYPE_C_LINKAGE)
  IF (HAVE_RETURN_TYPE_C_LINKAGE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-return-type-c-linkage")
  ENDIF(HAVE_RETURN_TYPE_C_LINKAGE)
ENDIF ()

# Google Flags
MESSAGE("-- Check for Google Flags")
FIND_LIBRARY(GFLAGS_LIB NAMES gflags PATHS ${SEARCH_LIBS})
IF (NOT EXISTS ${GFLAGS_LIB})
  MESSAGE(FATAL_ERROR
    "Can't find Google Flags. Please specify: "
    "-DGFLAGS_LIB=...")
ENDIF (NOT EXISTS ${GFLAGS_LIB})
MESSAGE("-- Found Google Flags library: ${GFLAGS_LIB}")
FIND_PATH(GFLAGS_INCLUDE NAMES gflags/gflags.h PATHS ${SEARCH_HEADERS})
IF (NOT EXISTS ${GFLAGS_INCLUDE})
  MESSAGE(FATAL_ERROR
    "Can't find Google Flags. Please specify: "
    "-DGFLAGS_INCLUDE=...")
ENDIF (NOT EXISTS ${GFLAGS_INCLUDE})
MESSAGE("-- Found Google Flags header in: ${GFLAGS_INCLUDE}")
INCLUDE_DIRECTORIES(${GFLAGS_INCLUDE})

# Don't search with REQUIRED as we can continue without gflags.
FIND_PACKAGE(Gflags REQUIRED)
IF (GFLAGS_FOUND)
  MESSAGE("-- Found Google Flags header in: ${GFLAGS_INCLUDE_DIRS}")
ELSE (GLOG_FOUND)
  MESSAGE(FATAL_ERROR "Can't find Google FLAGS.")
ENDIF (GFLAGS_FOUND)

# Google Logging
MESSAGE("-- Check for Google Log")
FIND_PACKAGE(Glog REQUIRED)
IF (GLOG_FOUND)
  MESSAGE("-- Found Google Log header in: ${GLOG_INCLUDE_DIRS}")
ELSE (GLOG_FOUND)
  MESSAGE(FATAL_ERROR "Can't find Google Log. Please set GLOG_INCLUDE_DIR & "
    "GLOG_LIBRARY")
ENDIF (GLOG_FOUND)

# LibCVD
FIND_PACKAGE(CVD REQUIRED)
IF (CVD_FOUND)
  MESSAGE("-- Found CVD header in: ${CVD_INCLUDE_DIRS}")
ELSE (CVD_FOUND)
  MESSAGE(FATAL_ERROR "Can't find LibCVD.")
ENDIF (CVD_FOUND)

INCLUDE_DIRECTORIES(
  include
  src
  ${CVD_INCLUDE_DIR}
  ${GFLAGS_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS})

IF (OPENMP)
  # Clang does not (yet) support OpenMP.
  IF (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    # Retain the help string associated with the OPENMP option
    # when updating it to disable use of OPENMP.
    GET_PROPERTY(HELP_STRING CACHE OPENMP PROPERTY HELPSTRING)
    SET(OPENMP OFF CACHE BOOL "${HELP_STRING}" FORCE)
    MESSAGE("-- Compiler is Clang, disabling OpenMP.")
  ELSE (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    # Find quietly s/t as we can continue without OpenMP if it is not found.
    FIND_PACKAGE(OpenMP QUIET)
    IF (OPENMP_FOUND)
      MESSAGE("-- Building with OpenMP.")
      ADD_DEFINITIONS(-DTHEIA_USE_OPENMP)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
      IF (UNIX)
        # At least on Linux, we need pthreads to be enabled for mutex to
        # compile.  This may not work on Windows or Android.
        FIND_PACKAGE(Threads REQUIRED)
        SET(STATIC_LIBRARY_FLAGS
          "${STATIC_LIBRARY_FLAGS} ${CMAKE_THREAD_LIBS_INIT}")
        SET(CMAKE_SHARED_LINKER_FLAGS
          "${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_THREAD_LIBS_INIT}")
      ENDIF (UNIX)
    ELSE (OPENMP_FOUND)
      MESSAGE("-- Failed to find OpenMP, disabling.")
      # Retain the help string associated with the OPENMP option
      # when updating it to disable use of OPENMP.
      GET_PROPERTY(HELP_STRING CACHE OPENMP PROPERTY HELPSTRING)
      SET(OPENMP OFF CACHE BOOL "${HELP_STRING}" FORCE)
    ENDIF (OPENMP_FOUND)
  ENDIF (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
ELSE (OPENMP)
  MESSAGE("-- Building without OpenMP.")
ENDIF (OPENMP)


# build Third party libraries included in distro.
ADD_SUBDIRECTORY(libraries)
INCLUDE_DIRECTORIES(
  libraries
  ${flann_SOURCE_DIR}/src/cpp
  ${gtest_SOURCE_DIR}/include
  ${gtest_SOURCE_DIR}
  ${hayai_SOURCE_DIR}
  ${vlfeat_SOURCE_DIR}
  ${agast_SOURCE_DIR})

INCLUDE("${CMAKE_MODULE_PATH}/OptimizeTheiaCompilerFlags.cmake")
OptimizeCompilerFlags()

ADD_SUBDIRECTORY(src/theia)

IF (BUILD_TUTORIALS)
  MESSAGE("-- Build the tutorials.")
  ADD_SUBDIRECTORY(tutorials)
ELSE (BUILD_TUTORIALS)
  MESSAGE("-- Do not build any tutorials.")
ENDIF (BUILD_TUTORIALS)

IF (BUILD_DOCUMENTATION)
  MESSAGE("-- Documentation building is enabled")

  # Generate the User's Guide (html).
  # The corresponding target is UserGuide, but is included in ALL.
  ADD_SUBDIRECTORY(docs)
ENDIF (BUILD_DOCUMENTATION)

# ---------------- Install commands ----------------- #
# Install the .h files
FILE(GLOB THEIA_HDRS ${CMAKE_SOURCE_DIR}/include/theia/*.h)
INSTALL(FILES ${THEIA_HDRS} DESTINATION include/theia)

FILE(GLOB_RECURSE THEIA_INTERNAL_HDRS ${CMAKE_SOURCE_DIR}/src/theia/*.h)
INSTALL(DIRECTORY src/theia/ DESTINATION include/theia FILES_MATCHING PATTERN "*.h")

# # Add an uninstall target to remove all installed files.
# CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/cmake/uninstall.cmake.in"
#                "${CMAKE_BINARY_DIR}/cmake/uninstall.cmake"
#                IMMEDIATE @ONLY)

# ADD_CUSTOM_TARGET(uninstall
#                   COMMAND ${CMAKE_COMMAND} -P ${CMAKE_BINARY_DIR}/cmake/uninstall.cmake)

# # Set up install directories. INCLUDE_INSTALL_DIR, LIB_INSTALL_DIR and
# # CMAKECONFIG_INSTALL_DIR must not be absolute paths.
# IF (WIN32)
#   SET(INCLUDE_INSTALL_DIR Include)
#   SET(LIB_INSTALL_DIR Lib)
#   SET(CMAKECONFIG_INSTALL_DIR CMake)
# ELSE ()
#   SET(INCLUDE_INSTALL_DIR include)
#   SET(LIB_INSTALL_DIR lib)
#   SET(CMAKECONFIG_INSTALL_DIR share/Theia)
# ENDIF ()

# # Figure out the relative path from the installed Config.cmake file to the
# # install prefix (which may be at runtime different from the chosen
# # CMAKE_INSTALL_PREFIX if under Windows the package was installed anywhere)
# # This relative path will be configured into the TheiaConfig.cmake.
# FILE(RELATIVE_PATH INSTALL_ROOT_REL_CONFIG_INSTALL_DIR
#      ${CMAKE_INSTALL_PREFIX}/${CMAKECONFIG_INSTALL_DIR} ${CMAKE_INSTALL_PREFIX})

# # Create a TheiaConfig.cmake file. <name>Config.cmake files are searched by
# # FIND_PACKAGE() automatically. We configure that file so that we can put any
# # information we want in it, e.g. version numbers, include directories, etc.
# CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/cmake/TheiaConfig.cmake.in"
#                "${CMAKE_CURRENT_BINARY_DIR}/TheiaConfig.cmake" @ONLY)

# # Additionally, when CMake has found a TheiaConfig.cmake, it can check for a
# # TheiaConfigVersion.cmake in the same directory when figuring out the version
# # of the package when a version has been specified in the FIND_PACKAGE() call,
# # e.g. FIND_PACKAGE(Theia [0.5.2] REQUIRED). The version argument is optional.
# CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/cmake/TheiaConfigVersion.cmake.in"
#                "${CMAKE_CURRENT_BINARY_DIR}/TheiaConfigVersion.cmake" @ONLY)

# # Install these files into the same directory as the generated exports-file,
# # we include the FindPackage scripts for libraries whose headers are included
# # in the public API of Theia and should thus be present in THEIA_INCLUDE_DIRS.
# INSTALL(FILES "${CMAKE_CURRENT_BINARY_DIR}/TheiaConfig.cmake"
#               "${CMAKE_CURRENT_BINARY_DIR}/TheiaConfigVersion.cmake"
#               "${CMAKE_SOURCE_DIR}/cmake/FindEigen.cmake"
#               "${CMAKE_SOURCE_DIR}/cmake/FindGlog.cmake"
#               "${CMAKE_SOURCE_DIR}/cmake/FindGflags.cmake"
#               "${CMAKE_SOURCE_DIR}/cmake/FindCVD.cmake"
#               DESTINATION ${CMAKECONFIG_INSTALL_DIR})
