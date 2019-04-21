#include "pose_rpy.h"


PoseRPY PoseRPY::operator-(const PoseRPY& other)
{
    PoseRPY result = *this;
    result.x -= other.x;
    result.y -= other.y;
    result.z -= other.z;
    result.rot_x -= other.rot_x;
    result.rot_y -= other.rot_y;
    result.rot_z -= other.rot_z;
    return result;
}


PoseRPY PoseRPY::operator+(const PoseRPY& other)
{
    PoseRPY result = *this;
    result.x += other.x;
    result.y += other.y;
    result.z += other.z;
    result.rot_x += other.rot_x;
    result.rot_y += other.rot_y;
    result.rot_z += other.rot_z;
    return result;
}


PoseRPY PoseRPY::operator*(const double& other)
{
    PoseRPY result = *this;
    result.x *= other;
    result.y *= other;
    result.z *= other;
    result.rot_x *= other;
    result.rot_y *= other;
    result.rot_z *= other;
    return result;
}


PoseRPY PoseRPY::operator/(const double& other)
{
    PoseRPY result = *this;
    result.x /= other;
    result.y /= other;
    result.z /= other;
    result.rot_x /= other;
    result.rot_y /= other;
    result.rot_z /= other;
    return result;
}


PoseRPY PoseRPY::make_pose_rpy(void)
{
    PoseRPY result;
    result.x = result.y = result.z = result.rot_x = result.rot_y = result.rot_z = 0;
    return result;
}


PoseRPY PoseRPY::transform_pose(const PoseRPY& pose, const double& rot_z)
{
    PoseRPY result = pose;
    result.x = pose.x * cos(rot_z) + pose.y * sin(rot_z),
    result.y = -pose.x * sin(rot_z) + pose.y * cos(rot_z);
    return result;
}


PoseRPY PoseRPY::transform_pose_3d(const PoseRPY& pose, const double& rot_x, const double& rot_y, const double& rot_z)
{
    PoseRPY result = transform_pose(pose, rot_z);
    double x = result.x, y = result.y, z = result.z;
    result.x = x * cos(rot_y) + sin(rot_y) * cos(rot_x) * z;
    result.y = y * cos(rot_x) + sin(rot_x) * cos(rot_y) * z;
    result.z = -x * sin(rot_y) - y * sin(rot_x) + z * cos(rot_x) * cos(rot_y); 
    return result;
}
