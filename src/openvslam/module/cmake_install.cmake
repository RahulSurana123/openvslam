# Install script for directory: /home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/module

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
   "/usr/local/include/openvslam/module/loop_bundle_adjuster.h;/usr/local/include/openvslam/module/frame_tracker.h;/usr/local/include/openvslam/module/loop_detector.h;/usr/local/include/openvslam/module/initializer.h;/usr/local/include/openvslam/module/local_map_cleaner.h;/usr/local/include/openvslam/module/relocalizer.h;/usr/local/include/openvslam/module/two_view_triangulator.h;/usr/local/include/openvslam/module/keyframe_inserter.h;/usr/local/include/openvslam/module/type.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/openvslam/module" TYPE FILE FILES
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/module/loop_bundle_adjuster.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/module/frame_tracker.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/module/loop_detector.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/module/initializer.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/module/local_map_cleaner.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/module/relocalizer.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/module/two_view_triangulator.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/module/keyframe_inserter.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/module/type.h"
    )
endif()

