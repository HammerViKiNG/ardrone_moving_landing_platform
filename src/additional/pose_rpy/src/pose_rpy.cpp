#include "pose_rpy.h"


PoseRPY::PoseRPY(const pose_rpy::PoseRPY& pose)
{
    this->x = pose.x;
    this->y = pose.y;
    this->z = pose.z;
    this->rot_x = pose.rot_x;
    this->rot_y = pose.rot_y;
    this->rot_z = pose.rot_z;
}


PoseRPY::PoseRPY(const geometry_msgs::Pose& pose)
{
    tf::quaternionMsgToTF(pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(temp_rpy[0], temp_rpy[1], temp_rpy[2]);
    this->x = pose.position.x;
    this->y = pose.position.y;
    this->z = pose.position.z;
    this->rot_x = temp_rpy[0];
    this->rot_y = temp_rpy[1];
    this->rot_z = temp_rpy[2];
}


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
