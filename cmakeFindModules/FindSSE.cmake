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
#
# Check if SSE instructions are available on the machine where
# the project is compiled.

MACRO(LIST_CONTAINS var value)
  SET(${var})
  FOREACH (value2 ${ARGN})
    IF (${value} STREQUAL ${value2})
      SET(${var} TRUE)
    ENDIF (${value} STREQUAL ${value2})
  ENDFOREACH (value2)
ENDMACRO(LIST_CONTAINS)


MACRO (FindSSE)

  IF(CMAKE_SYSTEM_NAME MATCHES "Linux")
    EXEC_PROGRAM(cat ARGS "/proc/cpuinfo" OUTPUT_VARIABLE CPUINFO_STRING)
    string(REPLACE " " ";" CPUINFO ${CPUINFO_STRING})

    LIST_CONTAINS(SSE2_TRUE "sse2" ${CPUINFO})
    IF (SSE2_TRUE)
      set(SSE2_FOUND true CACHE BOOL "SSE2 available on host")
    ELSE (SSE2_TRUE)
      set(SSE2_FOUND false CACHE BOOL "SSE2 available on host")
    ENDIF (SSE2_TRUE)

    # /proc/cpuinfo apparently omits sse3 :(
    LIST_CONTAINS(SSE3_TRUE "sse3" ${CPUINFO})
    IF (NOT SSE3_TRUE)
      STRING(REGEX REPLACE "^.*(T2300).*$" "¥¥1" SSE_THERE ${CPUINFO})
      STRING(COMPARE EQUAL "T2300" "${SSE_THERE}" SSE3_TRUE)
    ENDIF (NOT SSE3_TRUE)

    LIST_CONTAINS(SSSE3_TRUE "ssse3" ${CPUINFO})
    IF (SSE3_TRUE OR SSSE3_TRUE)
      set(SSE3_FOUND true CACHE BOOL "SSE3 available on host")
    ELSE (SSE3_TRUE OR SSSE3_TRUE)
      set(SSE3_FOUND false CACHE BOOL "SSE3 available on host")
    ENDIF (SSE3_TRUE OR SSSE3_TRUE)
    IF (SSSE3_TRUE)
      set(SSSE3_FOUND true CACHE BOOL "SSSE3 available on host")
    ELSE (SSSE3_TRUE)
      set(SSSE3_FOUND false CACHE BOOL "SSSE3 available on host")
    ENDIF (SSSE3_TRUE)

    LIST_CONTAINS(SSE41_TRUE "sse4_1" ${CPUINFO})
    IF (SSE41_TRUE)
      set(SSE4_1_FOUND true CACHE BOOL "SSE4.1 available on host")
    ELSE (SSE41_TRUE)
      set(SSE4_1_FOUND false CACHE BOOL "SSE4.1 available on host")
    ENDIF (SSE41_TRUE)

    LIST_CONTAINS(SSE42_TRUE "sse4_2" ${CPUINFO})
    IF (SSE42_TRUE)
      set(SSE4_2_FOUND true CACHE BOOL "SSE4.2 available on host")
    ELSE (SSE42_TRUE)
      set(SSE4_2_FOUND false CACHE BOOL "SSE4.2 available on host")
    ENDIF (SSE42_TRUE)

  ELSEIF(CMAKE_SYSTEM_NAME MATCHES "Darwin")
    EXEC_PROGRAM("/usr/sbin/sysctl -n machdep.cpu.features" OUTPUT_VARIABLE
      CPUINFO_STRING)
    string(REPLACE " " ";" CPUINFO ${CPUINFO_STRING})

    LIST_CONTAINS(SSE2_TRUE "SSE2" ${CPUINFO})
    IF (SSE2_TRUE)
      set(SSE2_FOUND true CACHE BOOL "SSE2 available on host")
    ELSE (SSE2_TRUE)
      set(SSE2_FOUND false CACHE BOOL "SSE2 available on host")
    ENDIF (SSE2_TRUE)

    LIST_CONTAINS(SSE3_TRUE "SSE3" ${CPUINFO})
    IF (SSE3_TRUE)
      set(SSE3_FOUND true CACHE BOOL "SSE3 available on host")
    ELSE (SSE3_TRUE)
      set(SSE3_FOUND false CACHE BOOL "SSE3 available on host")
    ENDIF (SSE3_TRUE)

    LIST_CONTAINS(SSSE3_TRUE "SSSE3" ${CPUINFO})
    IF (SSSE3_TRUE)
      set(SSSE3_FOUND true CACHE BOOL "SSSE3 available on host")
    ELSE (SSSE3_TRUE)
      set(SSSE3_FOUND false CACHE BOOL "SSSE3 available on host")
    ENDIF (SSSE3_TRUE)

    LIST_CONTAINS(SSE41_TRUE "SSE4.1" ${CPUINFO})
    IF (SSE41_TRUE)
      set(SSE4_1_FOUND true CACHE BOOL "SSE4.1 available on host")
    ELSE (SSE41_TRUE)
      set(SSE4_1_FOUND false CACHE BOOL "SSE4.1 available on host")
    ENDIF (SSE41_TRUE)

    LIST_CONTAINS(SSE42_TRUE "SSE4.2" ${CPUINFO})
    IF (SSE42_TRUE)
      set(SSE4_2_FOUND true CACHE BOOL "SSE4.2 available on host")
    ELSE (SSE42_TRUE)
      set(SSE4_2_FOUND false CACHE BOOL "SSE4.2 available on host")
    ENDIF (SSE42_TRUE)

  ELSEIF(CMAKE_SYSTEM_NAME MATCHES "Windows")
    # TODO
    set(SSE2_FOUND   true  CACHE BOOL "SSE2 available on host")
    set(SSE3_FOUND   false CACHE BOOL "SSE3 available on host")
    set(SSSE3_FOUND  false CACHE BOOL "SSSE3 available on host")
    set(SSE4_1_FOUND false CACHE BOOL "SSE4.1 available on host")
    set(SSE4_2_FOUND false CACHE BOOL "SSE4.2 available on host")
  ELSE(CMAKE_SYSTEM_NAME MATCHES "Linux")
    set(SSE2_FOUND   true  CACHE BOOL "SSE2 available on host")
    set(SSE3_FOUND   false CACHE BOOL "SSE3 available on host")
    set(SSSE3_FOUND  false CACHE BOOL "SSSE3 available on host")
    set(SSE4_1_FOUND false CACHE BOOL "SSE4.1 available on host")
    set(SSE4_2_FOUND false CACHE BOOL "SSE4.2 available on host")
  ENDIF(CMAKE_SYSTEM_NAME MATCHES "Linux")

  IF(CMAKE_COMPILER_IS_GNUCXX)
    EXECUTE_PROCESS(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    IF(GCC_VERSION VERSION_LESS 4.2)
      set(SSE4_1_FOUND false CACHE BOOL "SSE4.1 available on host" FORCE)
      set(SSE4_2_FOUND false CACHE BOOL "SSE4.2 available on host" FORCE)
    ENDIF()
  ENDIF(CMAKE_COMPILER_IS_GNUCXX)

  if(NOT SSE2_FOUND)
    MESSAGE(STATUS "Could not find support for SSE2 on this machine.")
  endif(NOT SSE2_FOUND)
  if(NOT SSE3_FOUND)
    MESSAGE(STATUS "Could not find support for SSE3 on this machine.")
  endif(NOT SSE3_FOUND)
  if(NOT SSSE3_FOUND)
    MESSAGE(STATUS "Could not find support for SSSE3 on this machine.")
  endif(NOT SSSE3_FOUND)
  if(NOT SSE4_1_FOUND)
    MESSAGE(STATUS "Could not find support for SSE4.1 on this machine.")
  endif(NOT SSE4_1_FOUND)
  if(NOT SSE4_2_FOUND)
    MESSAGE(STATUS "Could not find support for SSE4.2 on this machine.")
  endif(NOT SSE4_2_FOUND)

  mark_as_advanced(SSE2_FOUND SSE3_FOUND SSSE3_FOUND SSE4_1_FOUND SSE4_2_FOUND)

ENDMACRO(FindSSE)