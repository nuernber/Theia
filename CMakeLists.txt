CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0)

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

# Set c++ standard to c++0x
SET(CMAKE_CXX_FLAGS)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Multithreading using OpenMP
OPTION(OPENMP
       "Enable thread solving (requires OpenMP)"
       ON)
IF (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3")
ENDIF (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
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

# Macro for CC Libraries. If the target is header-only, use HEADER_LIBRARY macro
MACRO (CC_LIBRARY NAME SRCS)
  get_filename_component(TARGETDIR ${CMAKE_CURRENT_LIST_DIR} NAME)
  ADD_LIBRARY(${TARGETDIR}/${NAME} STATIC ${SRCS})
  SET_PROPERTY(TARGET ${TARGETDIR}/${NAME} PROPERTY OUTPUT_NAME ${NAME})
  TARGET_LINK_LIBRARIES(${TARGETDIR}/${NAME} ${ARGN})
ENDMACRO (CC_LIBRARY)

ADD_SUBDIRECTORY(solvers)
ADD_SUBDIRECTORY(vision)
