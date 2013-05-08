CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
IF (COMMAND cmake_policy)
  CMAKE_POLICY(SET CMP0003 NEW)
ENDIF (COMMAND cmake_policy)

PROJECT(THEIA C CXX)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Add GTest if testing is enabled
OPTION(BUILD_TESTING
       "Enable testing"
       ON)
ENABLE_TESTING()

# build Third party libraries
ADD_SUBDIRECTORY(libraries)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/libraries ${flann_SOURCE_DIR}/src/cpp ${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

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

# Force default build type to be Release
IF (NOT CMAKE_BUILD_TYPE)
  MESSAGE("-- No build type specified; defaulting to CMAKE_BUILD_TYPE=Release.")
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release."
    FORCE)
ELSE (NOT CMAKE_BUILD_TYPE)
  IF (CMAKE_BUILD_TYPE STREQUAL "Debug")
    MESSAGE("\n=================================================================================")
    MESSAGE("\n-- Build type: Debug. Performance will be terrible!")
    MESSAGE("-- Add -DCMAKE_BUILD_TYPE=Release to the CMake command line to get an optimized build.")
    MESSAGE("\n=================================================================================")
  ENDIF (CMAKE_BUILD_TYPE STREQUAL "Debug")
ENDIF (NOT CMAKE_BUILD_TYPE)

# Set c++ standard to c++11
SET(CMAKE_CXX_FLAGS)
#It requires C++11!
if(CMAKE_COMPILER_IS_GNUCXX)
   execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
   if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   elseif(GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3)
        message(WARNING "C++0x activated. If you get any errors update to a compiler which fully supports C++11")
        add_definitions("-std=gnu++0x")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
   else ()
        message(FATAL_ERROR "C++11 needed. Therefore a gcc compiler with a version higher than 4.3 is needed.")   
   endif()
else(CMAKE_COMPILER_IS_GNUCXX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif(CMAKE_COMPILER_IS_GNUCXX)

# Eigen
MESSAGE("-- Check for Eigen 3.0")
FIND_PATH(EIGEN_INCLUDE NAMES Eigen/Core PATHS ${EIGEN_SEARCH_HEADERS})
IF (NOT EXISTS ${EIGEN_INCLUDE})
  MESSAGE(FATAL_ERROR "Can't find Eigen. Try passing -DEIGEN_INCLUDE=...")
ENDIF (NOT EXISTS ${EIGEN_INCLUDE})
MESSAGE("-- Found Eigen 3.0: ${EIGEN_INCLUDE}")
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE})

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
# Create Theia dependency module.
# Required dependemcies.
SET(THEIA_LIBRARY_DEPENDENCIES ${THEIA_LIBRARY_DEPENDENCIES} ${GFLAGS_LIB})

# Google Logging
MESSAGE("-- Check for Google Log")
FIND_LIBRARY(GLOG_LIB NAMES glog PATHS ${SEARCH_LIBS})
IF (NOT EXISTS ${GLOG_LIB})
  MESSAGE(FATAL_ERROR
    "Can't find Google Log. Please specify: "
    "-DGLOG_LIB=...")
ENDIF (NOT EXISTS ${GLOG_LIB})
MESSAGE("-- Found Google Log library: ${GLOG_LIB}")

FIND_PATH(GLOG_INCLUDE NAMES glog/logging.h PATHS ${SEARCH_HEADERS})
IF (NOT EXISTS ${GLOG_INCLUDE})
  MESSAGE(FATAL_ERROR
    "Can't find Google Log. Please specify: "
    "-DGLOG_INCLUDE=...")
ENDIF (NOT EXISTS ${GLOG_INCLUDE})
MESSAGE("-- Found Google Log header in: ${GLOG_INCLUDE}")
INCLUDE_DIRECTORIES(${GLOG_INCLUDE})
SET(THEIA_LIBRARY_DEPENDENCIES ${THEIA_LIBRARY_DEPENDENCIES} ${GLOG_LIB})

# Multithreading using OpenMP
OPTION(OPENMP
       "Enable thread solving (requires OpenMP)"
       ON)
IF (${OPENMP})
  FIND_PACKAGE(OpenMP)
  IF(${OPENMP_FOUND})
    MESSAGE("-- Found OpenMP.")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    ADD_DEFINITIONS(-DTHEIA_USE_OPENMP)
    SET(THEIA_LIBRARY_DEPENDENCIES ${THEIA_LIBRARY_DEPENDENCIES} gomp)
  ELSE ({$OPENMP_FOUND})
    MESSAGE("-- Can't find OpenMP. Continuing without it.")
  ENDIF(${OPENMP_FOUND})
ENDIF (${OPENMP})

IF (CMAKE_BUILD_TYPE STREQUAL "Release")
  IF (CMAKE_COMPILER_IS_GNUCXX)
    # Linux
    IF (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
      SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mtune=native -msse2 -msse3 -msse4")
    ENDIF (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    # Mac OS X
    IF (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
      SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -msse4 -msse3 -msse2")
    ENDIF (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  ENDIF (CMAKE_COMPILER_IS_GNUCXX)
ENDIF (CMAKE_BUILD_TYPE STREQUAL "Release")

# Protocol buffers
OPTION(PROTOBUF
       "Enable protocol buffers support."
       ON)

IF (${PROTOBUF})
  FIND_PACKAGE(Protobuf)
  IF (${PROTOBUF_FOUND})
    #INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS})
    MESSAGE("-- Found Protocol Buffer library!")
    LIST(APPEND PROTOBUF_IMPORT_DIRS ${CMAKE_SOURCE_DIR})
    SET(THEIA_LIBRARY_DEPENDENCIES ${THEIA_LIBRARY_DEPENDENCIES} ${PROTOBUF_LIBRARY})
    #INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/internal)
  ELSE (${PROTOBUF_FOUND})
    ADD_DEFINITIONS(-DTHEIA_NO_PROTOCOL_BUFFERS)
  ENDIF (${PROTOBUF_FOUND})
ELSE (${PROTOBUF})
  ADD_DEFINITIONS(-DTHEIA_NO_PROTOCOL_BUFFERS)
ENDIF (${PROTOBUF})

# LibCVD
MESSAGE("-- Check for LibCVD")
FIND_LIBRARY(CVD_LIB NAMES cvd PATHS ${SEARCH_LIBS})
IF (NOT EXISTS ${CVD_LIB})
  MESSAGE(FATAL_ERROR
    "Can't find LibCVD. Please specify: "
    "-DCVD_LIB=...")
ENDIF (NOT EXISTS ${CVD_LIB})
MESSAGE("-- Found libCVD: ${CVD_LIB}")

FIND_PATH(CVD_INCLUDE NAMES cvd/image.h PATHS ${SEARCH_HEADERS})
IF (NOT EXISTS ${CVD_INCLUDE})
  MESSAGE(FATAL_ERROR
    "Can't find libCVD. Please specify: "
    "-DCVD_INCLUDE=...")
ENDIF (NOT EXISTS ${CVD_INCLUDE})
MESSAGE("-- Found libCVD header in: ${CVD_INCLUDE}")
INCLUDE_DIRECTORIES(${CVD_INCLUDE})
SET(THEIA_LIBRARY_DEPENDENCIES ${THEIA_LIBRARY_DEPENDENCIES} ${CVD_LIB})

# Macro for tests
#   Ex: Your test is written in myclass_test.cc  and uses a class in myclass.cc
#       which has been compiled as target: myclass then calling
#       GTEST(myclass_test myclass) will create a test executable myclass_test
#       by automatically compiling myclass_test.cc and linking the mytest target
MACRO (GTEST NAME)
  ADD_EXECUTABLE(${NAME} ${CMAKE_SOURCE_DIR}/test/test_main.cc ${NAME}.cc)
  TARGET_LINK_LIBRARIES(${NAME} gtest ${ARGN} ${THEIA_LIBRARY_DEPENDENCIES})
  ADD_TEST(NAME ${NAME}
           COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${NAME})
ENDMACRO (GTEST)

# Add c++ macro to get test data directory
ADD_DEFINITIONS(-DTHEIA_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/test/data")
SET(THEIA_SRC_FILES CACHE INTERNAL "include files" FORCE)
SET(THEIA_SRC_DEPS CACHE INTERNAL "include deps" FORCE)
# Macro for CC Libraries.
MACRO (CC_LIBRARY NAME SRCS)
  STRING(REPLACE "${CMAKE_SOURCE_DIR}/" "" TARGETDIR ${CMAKE_CURRENT_LIST_DIR})
  ADD_LIBRARY(${TARGETDIR}/${NAME} STATIC ${SRCS})
  SET_PROPERTY(TARGET ${TARGETDIR}/${NAME} PROPERTY OUTPUT_NAME ${NAME})
  SET_PROPERTY(TARGET ${TARGETDIR}/${NAME} PROPERTY LINKER_LANGUAGE CXX)
  TARGET_LINK_LIBRARIES(${TARGETDIR}/${NAME} ${ARGN} ${THEIA_LIBRARY_DEPENDENCIES})

  get_filename_component(ABS_PATH ${SRCS} ABSOLUTE)
  list(FIND THEIA_SRC_FILES ${ABS_PATH} _contains_already)
  MESSAGE("adding file to theia lib: ${ABS_PATH}")
  SET(THEIA_SRC_FILES ${THEIA_SRC_FILES} ${ABS_PATH} CACHE INTERNAL "include files")
  SET(THEIA_SRC_DEPS ${THEIA_SRC_DEPS} ${TARGETDIR}/${NAME} CACHE INTERNAL "include files")
ENDMACRO (CC_LIBRARY)

# Macro for Header Libraries.
MACRO (HEADER_LIBRARY NAME SRCS)
  STRING(REPLACE "${CMAKE_SOURCE_DIR}/" "" TARGETDIR ${CMAKE_CURRENT_LIST_DIR})
  ADD_LIBRARY(${TARGETDIR}/${NAME} STATIC ${CMAKE_SOURCE_DIR}/util/dummy.cc)
  SET_PROPERTY(TARGET ${TARGETDIR}/${NAME} PROPERTY OUTPUT_NAME ${NAME})
  SET_PROPERTY(TARGET ${TARGETDIR}/${NAME} PROPERTY LINKER_LANGUAGE CXX)
  TARGET_LINK_LIBRARIES(${TARGETDIR}/${NAME} ${ARGN} ${THEIA_LIBRARY_DEPENDENCIES})
ENDMACRO (HEADER_LIBRARY)

# Macro for CC Executables/Binaries
MACRO (CC_BINARY NAME)
  ADD_EXECUTABLE(${NAME} ${NAME}.cc)
  TARGET_LINK_LIBRARIES(${NAME} ${ARGN} ${THEIA_LIBRARY_DEPENDENCIES})
ENDMACRO (CC_BINARY)


function(THEIA_PROTOBUF_GENERATE_CPP SRCS HDRS)
  if(NOT ARGN)
    message(SEND_ERROR "Error: THEIA_PROTOBUF_GENERATE_CPP() called without any proto files")
    return()
  endif()

  if(DEFINED PROTOBUF_IMPORT_DIRS)
    foreach(DIR ${PROTOBUF_IMPORT_DIRS})
      get_filename_component(ABS_PATH ${DIR} ABSOLUTE)
      list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)
      if(${_contains_already} EQUAL -1)
          list(APPEND _protobuf_include_path -I ${ABS_PATH})
      endif()
    endforeach()
  endif()

  set(${SRCS})
  set(${HDRS})
  foreach(FIL ${ARGN})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)

    list(APPEND ${SRCS} "${CMAKE_CURRENT_SOURCE_DIR}/${FIL_WE}.pb.cc")
    list(APPEND ${HDRS} "${CMAKE_CURRENT_SOURCE_DIR}/${FIL_WE}.pb.h")
    MESSAGE("about to exec proto")
    add_custom_command(
      OUTPUT "${CMAKE_SOURCE_DIR}/${FIL_WE}.pb.cc"
             "${CMAKE_SOURCE_DIR}/${FIL_WE}.pb.h"
      COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --cpp_out  ${CMAKE_SOURCE_DIR} ${_protobuf_include_path} ${ABS_FIL}
      DEPENDS ${ABS_FIL}
      COMMENT "Running C++ protocol buffer compiler on ${FIL}"
      VERBATIM )
    MESSAGE("executed proto")
  endforeach()

  set_source_files_properties(${${SRCS}} ${${HDRS}} PROPERTIES GENERATED TRUE)
  set(${SRCS} ${${SRCS}} PARENT_SCOPE)
  set(${HDRS} ${${HDRS}} PARENT_SCOPE)
endfunction()


# Macro for protocol buffers
MACRO(PROTO_LIBRARY NAME SRCS)
  # If protobufs are enabled, compile them here.
  If (${PROTOBUF_FOUND})
    # Generate cpp files.
    THEIA_PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${SRCS})
    CC_LIBRARY(${NAME} ${PROTO_SRCS})
  # Otherwise, generate a dummy archive so that it will create a target without
  # consequence.
  ELSE (${PROTOBUF_FOUND})
    HEADER_LIBRARY(${NAME} dummy.h)
  ENDIF (${PROTOBUF_FOUND})
ENDMACRO(PROTO_LIBRARY)



# Include all source directories.
ADD_SUBDIRECTORY(image)
ADD_SUBDIRECTORY(math)
ADD_SUBDIRECTORY(solvers)
ADD_SUBDIRECTORY(vision)

# Compile protos first if desired.
IF(${PROTOBUF_FOUND})
  file(GLOB_RECURSE ProtoFiles "${CMAKE_SOURCE_DIR}/*.proto")
  FOREACH(PROTO_FILE ${ProtoFiles})
    MESSAGE("proto files = ${PROTO_FILE}")
    THEIA_PROTOBUF_GENERATE_CPP(ProtoSource ProtoHeader ${PROTO_FILE})
  ENDFOREACH(PROTO_FILE)
ENDIF(${PROTOBUF_FOUND})

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

# Install the .h files
install(DIRECTORY ${CMAKE_SOURCE_DIR}/theia DESTINATION include
  FILES_MATCHING PATTERN "*.h")

# Install the library
FOREACH(THEIA_FILE ${THEIA_SRC_FILES})
  MESSAGE("theia files = ${THEIA_FILE}")
ENDFOREACH(THEIA_FILE)

ADD_LIBRARY(theia STATIC ${THEIA_SRC_FILES})
TARGET_LINK_LIBRARIES(theia ${THEIA_LIBRARY_DEPENDENCIES} ${THEIA_SRC_DEPS})
INSTALL(TARGETS theia DESTINATION lib)

