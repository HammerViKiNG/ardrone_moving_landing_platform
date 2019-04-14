#ifndef POSE_RPY_H
#define POSE_RPY_H

#include <cmath>

struct PoseRPY
{
    double x;
    double y;
    double z;
    double rot_x;
    double rot_y;
    double rot_z;

    PoseRPY operator-(const PoseRPY& other);
    PoseRPY operator+(const PoseRPY& other);
    PoseRPY operator*(const double& other);
    PoseRPY operator/(const double& other);

    static PoseRPY make_pose_rpy(void);

    static PoseRPY transform_pose(const PoseRPY& pose, const double& rot_z); 
};


#endif
