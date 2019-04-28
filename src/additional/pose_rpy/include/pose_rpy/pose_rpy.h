#ifndef POSE_RPY_H
#define POSE_RPY_H

#include "pose_rpy/PoseRPY.h"
#include "tf/transform_datatypes.h"
#include <cmath>

struct PoseRPY : pose_rpy::PoseRPY
{
    PoseRPY() {};
    PoseRPY(const geometry_msgs::Pose& pose);
    PoseRPY(const pose_rpy::PoseRPY& pose);

    PoseRPY operator-(const PoseRPY& other);
    PoseRPY operator+(const PoseRPY& other);
    PoseRPY operator*(const double& other);
    PoseRPY operator/(const double& other);

    static PoseRPY get_pose_rpy(const geometry_msgs::Pose& pose);
    static PoseRPY get_pose_rpy(const pose_rpy::PoseRPY& pose);

    static PoseRPY transform_pose(const PoseRPY& pose, const double& rot_z); 
    static PoseRPY transform_pose_3d(const PoseRPY& pose, const double& rot_x, const double& rot_y, const double& rot_z);

    private:

        static tf::Quaternion quat; 
        static double temp_rpy[2]; 

};

tf::Quaternion PoseRPY::quat;
double PoseRPY::temp_rpy[2];

#endif
