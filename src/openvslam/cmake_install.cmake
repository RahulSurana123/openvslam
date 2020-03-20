# Install script for directory: /home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam

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
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libopenvslam.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libopenvslam.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/lib/libopenvslam.so"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libopenvslam.so")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE SHARED_LIBRARY FILES "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/lib/libopenvslam.so")
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/libopenvslam.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/lib/libopenvslam.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/local/lib/libopenvslam.so"
         OLD_RPATH "/opt/ros/kinetic/lib/x86_64-linux-gnu:/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/lib/libopenvslam.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/openvslam/config.h;/usr/local/include/openvslam/tracking_module.h;/usr/local/include/openvslam/mapping_module.h;/usr/local/include/openvslam/global_optimization_module.h;/usr/local/include/openvslam/type.h;/usr/local/include/openvslam/system.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/openvslam" TYPE FILE FILES
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/config.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/tracking_module.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/mapping_module.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/global_optimization_module.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/type.h"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/system.h"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/openvslam/3rd/json/include/nlohmann/json.hpp;/usr/local/include/openvslam/3rd/json/include/nlohmann/json_fwd.hpp")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/openvslam/3rd/json/include/nlohmann" TYPE FILE FILES
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/3rd/json/include/nlohmann/json.hpp"
    "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/3rd/json/include/nlohmann/json_fwd.hpp"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/openvslam/3rd/spdlog/include")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/openvslam/3rd/spdlog" TYPE DIRECTORY FILES "/home/meditab/catkin_ws/src/ros_openvslam/openvslam/3rd/spdlog/include")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/camera/cmake_install.cmake")
  include("/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/data/cmake_install.cmake")
  include("/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/feature/cmake_install.cmake")
  include("/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/initialize/cmake_install.cmake")
  include("/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/io/cmake_install.cmake")
  include("/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/match/cmake_install.cmake")
  include("/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/module/cmake_install.cmake")
  include("/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/optimize/cmake_install.cmake")
  include("/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/publish/cmake_install.cmake")
  include("/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/solve/cmake_install.cmake")
  include("/home/meditab/catkin_ws/src/ros_openvslam/openvslam/src/openvslam/util/cmake_install.cmake")

endif()

