CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

IF (COMMAND cmake_policy)
  CMAKE_POLICY(SET CMP0003 NEW)
ENDIF (COMMAND cmake_policy)

PROJECT(THEIA C CXX)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Add GTest as a target.
OPTION(BUILD_TESTING
       "Enable testing"
       ON)
ENABLE_TESTING()
IF (${BUILD_TESTING})
  # TODO(cmsweeney): change this or include gtest in the distribution
  ADD_SUBDIRECTORY(libraries/gtest)
  INCLUDE_DIRECTORIES(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
ENDIF (${BUILD_TESTING})

# Make libraries more static.
#SET(CMAKE_FIND_LIBRARY_SUFFIXES .a ${CMAKE_FIND_LIBRARY_SUFFIXES})

# Default locations to search for on various platforms.
LIST(APPEND SEARCH_LIBS /usr/lib)
LIST(APPEND SEARCH_LIBS /usr/local/lib)
LIST(APPEND SEARCH_LIBS /usr/local/homebrew/lib) # Mac OS X
LIST(APPEND SEARCH_LIBS /opt/local/lib)

LIST(APPEND SEARCH_HEADERS /usr/include)
LIST(APPEND SEARCH_HEADERS /usr/local/include)
LIST(APPEND SEARCH_HEADERS /usr/local/homebrew/include) # Mac OS X
LIST(APPEND SEARCH_HEADERS /opt/local/include)

# Locations to search for Eigen
SET(EIGEN_SEARCH_HEADERS ${SEARCH_HEADERS})
LIST(APPEND EIGEN_SEARCH_HEADERS /usr/include/eigen3) # Ubuntu 10.04's default location.
LIST(APPEND EIGEN_SEARCH_HEADERS /usr/local/include/eigen3)
LIST(APPEND EIGEN_SEARCH_HEADERS /usr/local/homebrew/include/eigen3)  # Mac OS X
LIST(APPEND EIGEN_SEARCH_HEADERS /opt/local/var/macports/software/eigen3/opt/local/include/eigen3) # Mac OS X

# Set c++ standard to c++0x
SET(CMAKE_CXX_FLAGS)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Eigen
MESSAGE("-- Check for Eigen 3.0")
FIND_PATH(EIGEN_INCLUDE NAMES Eigen/Core PATHS ${EIGEN_SEARCH_HEADERS})
IF (NOT EXISTS ${EIGEN_INCLUDE})
  MESSAGE(FATAL_ERROR "Can't find Eigen. Try passing -DEIGEN_INCLUDE=...")
ENDIF (NOT EXISTS ${EIGEN_INCLUDE})
MESSAGE("-- Found Eigen 3.0: ${EIGEN_INCLUDE}")
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE})

# Multithreading using OpenMP
OPTION(OPENMP
       "Enable thread solving (requires OpenMP)"
       ON)
IF (${OPENMP})
  FIND_PACKAGE(OpenMP)
  IF(${OPENMP_FOUND})
    MESSAGE("-- Found OpenMP.")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    ADD_DEFINITIONS(-DUSE_OPENMP)
  ELSE ({$OPENMP_FOUND})
    MESSAGE("-- Can't find OpenMP. Continuing without it.")
  ENDIF(${OPENMP_FOUND})
ENDIF (${OPENMP})

IF (CMAKE_BUILD_TYPE STREQUAL "Release")
  IF (CMAKE_COMPILER_IS_GNUCXX)
    # Linux
    IF (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
      SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mtune=native")
    ENDIF (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    # Mac OS X
    IF (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
      SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -msse3")
    ENDIF (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  ENDIF (CMAKE_COMPILER_IS_GNUCXX)
ENDIF (CMAKE_BUILD_TYPE STREQUAL "Release")

# Macro for tests
#   Ex: Your test is written in myclass_test.cc  and uses a class in myclass.cc
#       which has been compiled as target: myclass then calling
#       GTEST(myclass_test myclass) will create a test executable myclass_test
#       by automatically compiling myclass_test.cc and linking the mytest target
MACRO (GTEST NAME)
  ADD_EXECUTABLE(${NAME} ${NAME}.cc)
  TARGET_LINK_LIBRARIES(${NAME} gtest gtest_main ${ARGN})
  ADD_TEST(NAME ${NAME}
           COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${NAME}
           --test_srcdir
           ${CMAKE_SOURCE_DIR}/data)
ENDMACRO (GTEST)

# Macro for CC Libraries.
MACRO (CC_LIBRARY NAME SRCS)
  STRING(REPLACE "${CMAKE_SOURCE_DIR}/" "" TARGETDIR ${CMAKE_CURRENT_LIST_DIR})
  ADD_LIBRARY(${TARGETDIR}/${NAME} STATIC ${SRCS})
  SET_PROPERTY(TARGET ${TARGETDIR}/${NAME} PROPERTY OUTPUT_NAME ${NAME})
  TARGET_LINK_LIBRARIES(${TARGETDIR}/${NAME} ${ARGN})
ENDMACRO (CC_LIBRARY)

ADD_SUBDIRECTORY(math)
ADD_SUBDIRECTORY(solvers)
ADD_SUBDIRECTORY(vision)
