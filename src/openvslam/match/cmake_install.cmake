# Install script for directory: /home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/match

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/openvslam/match/angle_checker.h;/usr/local/include/openvslam/match/bow_tree.h;/usr/local/include/openvslam/match/stereo.h;/usr/local/include/openvslam/match/projection.h;/usr/local/include/openvslam/match/base.h;/usr/local/include/openvslam/match/area.h;/usr/local/include/openvslam/match/fuse.h;/usr/local/include/openvslam/match/robust.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/openvslam/match" TYPE FILE FILES
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/match/angle_checker.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/match/bow_tree.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/match/stereo.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/match/projection.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/match/base.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/match/area.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/match/fuse.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/match/robust.h"
    )
endif()

