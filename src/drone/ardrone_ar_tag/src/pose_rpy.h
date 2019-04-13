#ifndef POSE_RPY_H
#define POSE_RPY_H


#include <ros/ros.h>

struct PoseRPY
{
    double x;
    double y;
    double z;
    double rot_x;
    double rot_y;
    double rot_z;

    PoseRPY operator-(const PoseRPY& other);
};


#endif
