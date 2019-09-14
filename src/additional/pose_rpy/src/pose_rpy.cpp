#include "pose_rpy/pose_rpy.h"


tf::Quaternion PoseRPY::quat; 


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


PoseRPY PoseRPY::operator=(const PoseRPY& other)
{
    this->x = other.x;
    this->y = other.y;
    this->z = other.z;
    this->rot_x = other.rot_x;
    this->rot_y = other.rot_y;
    this->rot_z = other.rot_z;
    return *this;
}


PoseRPY PoseRPY::zero_pose_rpy(void)
{
    PoseRPY result;
    result.x = result.y = result.z = result.rot_x = result.rot_y = result.rot_z = 0;
    return result;
}


PoseRPY PoseRPY::get_pose_rpy(const geometry_msgs::Pose& pose)
{
    PoseRPY result;
    tf::quaternionMsgToTF(pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(result.rot_x, result.rot_y, result.rot_z);
    result.x = pose.position.x;
    result.y = pose.position.y;
    result.z = pose.position.z;
    return result;
}


PoseRPY PoseRPY::transform_pose(const PoseRPY& pose, const double& rot_z)
{
    PoseRPY result = pose;
    result.x = pose.x * cos(rot_z) + pose.y * sin(rot_z);
    result.y = -pose.x * sin(rot_z) + pose.y * cos(rot_z);
    result.rot_z -= rot_z;
    return result;
}


PoseRPY PoseRPY::transform_pose_zyx(const PoseRPY& pose, const double& rot_x, const double& rot_y, const double& rot_z)
{
    PoseRPY result = pose;

    tf::Quaternion rotation;
    rotation.setRPY(rot_x, rot_y, rot_z);

    tf::Vector3 pose_vec(pose.x, pose.y, pose.z);
    pose_vec = tf::quatRotate(rotation, pose_vec);
    result.x = pose_vec.x();
    result.y = pose_vec.y();
    result.z = pose_vec.z();

    tf::Quaternion original;
    original.setRPY(pose.rot_x, pose.rot_y, pose.rot_z);
    //ROS_INFO("%f, %f, %f", original.x(), original.y(), original.z());

    original = original * rotation;

    tf::Matrix3x3(original).getRPY(result.rot_x, result.rot_y, result.rot_z);

    return result;
}


<<<<<<< HEAD
/*PoseRPY PoseRPY::transform_pose_zyx(const PoseRPY& pose, const double& rot_x, const double& rot_y, const double& rot_z)
=======
PoseRPY PoseRPY::transform_pose_zyx(const PoseRPY& pose, const double& rot_x, const double& rot_y, const double& rot_z)
>>>>>>> 5fc08fe4a1a040212463e765df79f00ca72d8f13
{
    PoseRPY result = transform_pose(pose, rot_z);
    double x = result.x, y = result.y, z = result.z;
    result.x = x * cos(rot_y) + sin(rot_y) * cos(rot_x) * z;
    result.y = y * cos(rot_x) - sin(rot_x) * cos(rot_y) * z;
    //result.x = x * cos(rot_y) + sin(rot_y) * z;
    //result.y = y * cos(rot_x) - sin(rot_x) * z;
<<<<<<< HEAD
    result.z = -x * sin(rot_y) - y * sin(rot_x) + z * cos(rot_x) * cos(rot_y);
    return result;
}*/
=======
    result.z = -x * sin(rot_y) - y * sin(rot_x) + z * cos(rot_x) * cos(rot_y); 
    return result;
}
>>>>>>> 5fc08fe4a1a040212463e765df79f00ca72d8f13


/*PoseRPY PoseRPY::transform_pose_zyx(const PoseRPY& pose, const double& rot_x, const double& rot_y, const double& rot_z)
{
    PoseRPY result = pose;
    Eigen::Vector3f vec;
    vec << result.x, result.y, result.z;
    Eigen::Matrix3f mat;
    mat = Eigen::AngleAxisf(result.rot_x, Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(result.rot_y, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(result.rot_z, Eigen::Vector3f::UnitZ());
    vec = mat * vec;
    result.x = vec[0];
    result.y = vec[1];
    result.z = vec[2];
    return result;
}*/


