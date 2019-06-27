#ifndef POSE_RPY_H
#define POSE_RPY_H

#include "tf/transform_datatypes.h"
#include <cmath>


struct PoseRPY 
{
    double x, y, z;
    double rot_x, rot_y, rot_z;

    PoseRPY operator-(const PoseRPY& other);
    PoseRPY operator+(const PoseRPY& other);
    PoseRPY operator*(const double& other);
    PoseRPY operator/(const double& other);

    static tf::Quaternion quat;  

    static PoseRPY zero_pose_rpy(void);

    static PoseRPY get_pose_rpy(const geometry_msgs::Pose& pose);

    static PoseRPY transform_pose(const PoseRPY& pose, const double& rot_z); 
    static PoseRPY transform_pose_3d(const PoseRPY& pose, const double& rot_x, const double& rot_y, const double& rot_z);

};

#endif
