# - Try to find libCVD
#
#  CVD_FOUND - system has libCVD
#  CVD_INCLUDE_DIR - the libCVD include directories
#  CVD_LIBRARIES - link these to use libCVD

FIND_PATH(
  CVD_INCLUDE_DIR
  NAMES cvd/image.h
  PATHS
    ${CMAKE_SOURCE_DIR}/../libcvd
    ${CMAKE_SOURCE_DIR}/../cvd
    /usr/include
    /usr/local/include
)

FIND_LIBRARY(
  CVD_LIBRARIES
  cvd
  PATHS
    ${CMAKE_SOURCE_DIR}/../libcvd
    ${CMAKE_SOURCE_DIR}/../cvd
    /usr/lib
    /usr/local/lib
)

IF(CVD_INCLUDE_DIR AND CVD_LIBRARIES)
  SET(CVD_FOUND TRUE)
ENDIF()

IF(CVD_FOUND)
   IF(NOT CVD_FIND_QUIETLY)
      MESSAGE(STATUS "Found CVD: ${CVD_LIBRARIES}")
   ENDIF()
ELSE()
   IF(CVD_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find CVD")
   ENDIF()
ENDIF()