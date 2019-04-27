#ifndef POSE_RPY_H
#define POSE_RPY_H

#include "pose_rpy/PoseRPY.h"
#include "tf/transform_datatypes.h"
#include <cmath>

struct PoseRPY : pose_rpy::PoseRPY
{
    PoseRPY(const geometry_msgs::Pose& pose);
    PoseRPY(const pose_rpy::PoseRPY& pose);

    PoseRPY operator-(const PoseRPY& other);
    PoseRPY operator+(const PoseRPY& other);
    PoseRPY operator*(const double& other);
    PoseRPY operator/(const double& other);

    static PoseRPY transform_pose(const PoseRPY& pose, const double& rot_z); 
    static PoseRPY transform_pose_3d(const PoseRPY& pose, const double& rot_x, const double& rot_y, const double& rot_z); 

    static tf::Quaternion quat; 
};


#endif
