# These two methods are macros so that they can modify global parameters
# (functions cannot do that easily with CMake)

macro(set_compile_parameters)
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

  # Release compile parameters and optimizations.
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
endmacro(set_compile_parameters)

macro(find_theia_dependencies)
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
  # Required dependemcies
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

  # Protocol buffers
  OPTION(PROTOBUF
    "Enable protocol buffers support."
    ON)
  IF (${PROTOBUF})
    FIND_PACKAGE(Protobuf)
    IF (${PROTOBUF_FOUND})
      INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIRS})
      MESSAGE("-- Found Protocol Buffer library!")
      LIST(APPEND PROTOBUF_IMPORT_DIRS ${CMAKE_SOURCE_DIR})
      SET(THEIA_LIBRARY_DEPENDENCIES ${THEIA_LIBRARY_DEPENDENCIES} ${PROTOBUF_LIBRARY})
      INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/internal)
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

endmacro(find_theia_dependencies)