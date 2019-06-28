# ardrone_moving_landing_platform
A moving landing platform for AR.drone 2.0 for Ubunutu 16.04, ROS Kinetic and Gazebo 7. 

The system consists of an EV3 robot with printed AR tag and AR.drone 2.0.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Requirements

Ubuntu 16.04;
ROS Kinetic;
Gazebo 7.

### Installation

#### From requirements.bash
Clone the project:
```
git clone https://github.com/HammerViKiNG/ardrone_moving_landing_platform.git
```
Run the requirements.bash file:
```
bash requirements.bash
```

#### From sources 

First of all, you need to create a catkin workspace:
```
cd ~
mkdir robotic_software
cd robotic_software
```

Clone the project:
```
git clone https://github.com/HammerViKiNG/ardrone_moving_landing_platform.git
```

Install ardrone autonomy from ROS repository:
```
sudo apt-get ros-kinetic-ardrone-autonomy
```

or from source:
```
cd ~/robotic_software/ardrone_moving_landing_platform
git clone https://github.com/AutonomyLab/ardrone_autonomy.git -b indigo-devel
cd ~/robotic_software/ardrone_moving_landing_platform/
rosdep install --from-paths src -i
```

Install compiled package ar-track-alvar from ROS repository:
```
sudo apt-get ros-kinetic-ar-track-alvar
```

Install AR.drone-ROS package for simulation:
```
cd src
git clone https://github.com/eborghi10/AR.Drone-ROS.git
```

Build the package
```
catkin_make
```

## Running the tests

For testing of system, after build of package run:
```
roslaunch simulation testworld_ar.launch
```

## Launch from the Docker container

Install Docker CE: [https://docs.docker.com/install/linux/docker-ce/ubuntu/](https://docs.docker.com/install/linux/docker-ce/ubuntu/)

Pull the ROS Kinetic image:
```
sudo docker pull osrf/ros:kinetic-desktop-full
```

Build the container:
```
sudo docker build . --tag=ardrone_image
```

Run the container:
```
sudo docker run -it --env=DISPLAY ardrone_tag
```

# Built with

* [ardrone_autonomy](http://wiki.ros.org/ardrone_autonomy) - driver for AR.Drone 2.0;
* [AR.Drone-ROS](https://github.com/eborghi10/AR.Drone-ROS) - porting of tum_simulator on ROS Kinetic for simulation of AR.Drone 2.0 in Gazebo;
* [ar_track_alvar](http://wiki.ros.org/ar_track_alvar) - package for detection of AR tags by camera.
