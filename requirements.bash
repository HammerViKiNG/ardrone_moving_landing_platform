#!/bin/bash

# Create catkin workspace
cd ~
mkdir robotic_software
cd robotic_software

# Clone project
git clone https://github.com/HammerViKiNG/ardrone_moving_landing_platform.git

# Install ardrone autonomy from ROS repository
sudo apt-get ros-kinetic-ardrone-autonomy
# or from source 
# cd ~/robotic_software/ardrone_moving_landing_platform
# git clone https://github.com/AutonomyLab/ardrone_autonomy.git -b indigo-devel
# cd ~/robotic_software/ardrone_moving_landing_platform/
# rosdep install --from-paths src -i

# install AR.drone-ROS package for simulation:
cd src
git clone https://github.com/eborghi10/AR.Drone-ROS.git

# Install compiled package ar-track-alvar from ROS repository
sudo apt-get ros-kinetic-ar-track-alvar

# Build the package
catkin_make


