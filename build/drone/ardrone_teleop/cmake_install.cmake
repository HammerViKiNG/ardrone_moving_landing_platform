# Install script for directory: /home/hammerviking/robotic_software/ardrone_moving_landing_platform/src/drone/ardrone_teleop

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/hammerviking/robotic_software/ardrone_moving_landing_platform/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_teleop/cmake" TYPE FILE FILES "/home/hammerviking/robotic_software/ardrone_moving_landing_platform/build/drone/ardrone_teleop/catkin_generated/installspace/ardrone_teleop-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/hammerviking/robotic_software/ardrone_moving_landing_platform/devel/share/roseus/ros/ardrone_teleop")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/hammerviking/robotic_software/ardrone_moving_landing_platform/devel/lib/python2.7/dist-packages/ardrone_teleop")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/hammerviking/robotic_software/ardrone_moving_landing_platform/devel/lib/python2.7/dist-packages/ardrone_teleop")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/hammerviking/robotic_software/ardrone_moving_landing_platform/build/drone/ardrone_teleop/catkin_generated/installspace/ardrone_teleop.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_teleop/cmake" TYPE FILE FILES "/home/hammerviking/robotic_software/ardrone_moving_landing_platform/build/drone/ardrone_teleop/catkin_generated/installspace/ardrone_teleop-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_teleop/cmake" TYPE FILE FILES
    "/home/hammerviking/robotic_software/ardrone_moving_landing_platform/build/drone/ardrone_teleop/catkin_generated/installspace/ardrone_teleopConfig.cmake"
    "/home/hammerviking/robotic_software/ardrone_moving_landing_platform/build/drone/ardrone_teleop/catkin_generated/installspace/ardrone_teleopConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ardrone_teleop" TYPE FILE FILES "/home/hammerviking/robotic_software/ardrone_moving_landing_platform/src/drone/ardrone_teleop/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ardrone_teleop" TYPE PROGRAM FILES "/home/hammerviking/robotic_software/ardrone_moving_landing_platform/build/drone/ardrone_teleop/catkin_generated/installspace/keyboard_teleop.py")
endif()

