# Install script for directory: /home/siqi/Documents/robia/robia/vq2-1.33/src

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "devel")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/include/vq2/vq2Proba.h;/usr/include/vq2/vq2Concept.h;/usr/include/vq2/vq2Graph.h;/usr/include/vq2/vq2Memory.h;/usr/include/vq2/vq2ByDefault.h;/usr/include/vq2/vq2Functors.h;/usr/include/vq2/vq2SOM.h;/usr/include/vq2/vq2Fig.h;/usr/include/vq2/vq2.h;/usr/include/vq2/vq2Closest.h;/usr/include/vq2/vq2GNGT.h;/usr/include/vq2/vq2KMeans.h;/usr/include/vq2/vq2Time.h;/usr/include/vq2/vq2Speed.h;/usr/include/vq2/vq2Unit.h;/usr/include/vq2/vq2Topology.h")
  IF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
  IF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
FILE(INSTALL DESTINATION "/usr/include/vq2" TYPE FILE FILES
    "/home/siqi/Documents/robia/robia/vq2-1.33/src/vq2Proba.h"
    "/home/siqi/Documents/robia/robia/vq2-1.33/src/vq2Concept.h"
    "/home/siqi/Documents/robia/robia/vq2-1.33/src/vq2Graph.h"
    "/home/siqi/Documents/robia/robia/vq2-1.33/src/vq2Memory.h"
    "/home/siqi/Documents/robia/robia/vq2-1.33/src/vq2ByDefault.h"
    "/home/siqi/Documents/robia/robia/vq2-1.33/src/vq2Functors.h"
    "/home/siqi/Documents/robia/robia/vq2-1.33/src/vq2SOM.h"
    "/home/siqi/Documents/robia/robia/vq2-1.33/src/vq2Fig.h"
    "/home/siqi/Documents/robia/robia/vq2-1.33/src/vq2.h"
    "/home/siqi/Documents/robia/robia/vq2-1.33/src/vq2Closest.h"
    "/home/siqi/Documents/robia/robia/vq2-1.33/src/vq2GNGT.h"
    "/home/siqi/Documents/robia/robia/vq2-1.33/src/vq2KMeans.h"
    "/home/siqi/Documents/robia/robia/vq2-1.33/src/vq2Time.h"
    "/home/siqi/Documents/robia/robia/vq2-1.33/src/vq2Speed.h"
    "/home/siqi/Documents/robia/robia/vq2-1.33/src/vq2Unit.h"
    "/home/siqi/Documents/robia/robia/vq2-1.33/src/vq2Topology.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "devel")

