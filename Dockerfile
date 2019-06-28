FROM osrf/ros:kinetic-desktop-full
MAINTAINER hammerviking
RUN apt-get update 
RUN apt-get install -y apt-utils ros-kinetic-ardrone-autonomy ros-kinetic-ar-track-alvar
WORKDIR /home
COPY . /home
RUN rm -rf devel build

ENV QT_X11_NO_MITSHM 1
VOLUME /tmp/.X11-unix:/tmp/.X11-unix:rw
