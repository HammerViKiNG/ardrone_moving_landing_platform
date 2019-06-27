#include "ardrone_pose_handler/ardrone_pose_handler.h"


ArdronePoseHandler::ArdronePoseHandler(std::string navdata_topic)
{
    sub_navdata = nh.subscribe(navdata_topic, 1, &ArdronePoseHandler::navdata_callback, this);
    sub_navdata = nh.subscribe("/ardrone/imu", 1, &ArdronePoseHandler::imu_callback, this);
    boost::shared_ptr<ardrone_autonomy::Navdata const> start_data = ros::topic::waitForMessage<ardrone_autonomy::Navdata>(navdata_topic, nh);
    last_time = ros::Time::now();
    pose_rpy.x = 0; 
    pose_rpy.y = 0;
    pose_rpy.z = start_data->altd / 1000.0;
    pose_rpy.rot_x = start_data->rotX * M_PI / 180.0;
    pose_rpy.rot_y = start_data->rotY * M_PI / 180.0;
    pose_rpy.rot_z = start_data->rotZ * M_PI / 180.0;
}


PoseRPY ArdronePoseHandler::local_to_global(const PoseRPY& pose)
{
    return PoseRPY::transform_pose(pose, -pose_rpy.rot_z);
}


PoseRPY ArdronePoseHandler::global_to_local(const PoseRPY& pose)
{
    return PoseRPY::transform_pose(pose, pose_rpy.rot_z);
}


PoseRPY ArdronePoseHandler::local_to_global_shifted(PoseRPY pose, const double& rot_z)
{
    pose.x += 0.15;
    return PoseRPY::transform_pose(pose, -rot_z);
}


PoseRPY ArdronePoseHandler::global_to_local_shifted(PoseRPY pose, const double& rot_z)
{
    PoseRPY result = PoseRPY::transform_pose(pose, rot_z);
    result.x -= 0.15;
    return result;
}


std::pair<double, double> ArdronePoseHandler::global_to_local(double x, double y, double rot_z)
{
    return std::make_pair<double, double>(x * cos(rot_z) + y * sin(rot_z),
                                          -x * sin(rot_z) + y * cos(rot_z));   
}


void ArdronePoseHandler::navdata_callback(const ardrone_autonomy::Navdata& msg)
{
    pose_rpy.rot_x = msg.rotX * M_PI / 180.0;
    pose_rpy.rot_y = msg.rotY * M_PI / 180.0;
    pose_rpy.rot_z = msg.rotZ * M_PI / 180.0;
    pose_rpy.z = msg.altd / 1000.0;
    /*double curr_time = msg.tm / 1000000.0;
    odometry(msg.vx, msg.vy, curr_time - last_time);
    std::pair<double, double> v_global = global_to_local(msg.vx, msg.vy, -pose_rpy.rot_z);
    velocity.x = v_global.first;
    velocity.y = v_global.second; */
    velocity.z = msg.vz;
    state = msg.state;
    //last_time = curr_time; 
}


void ArdronePoseHandler::imu_callback(const sensor_msgs::Imu& msg)
{
    ros::Time curr_time = ros::Time::now();
    double ax = msg.linear_acceleration.x,
           ay = msg.linear_acceleration.y,
           dt = (curr_time - last_time).toNSec() / 1e+9;
    velocity.x += ((cos(pose_rpy.rot_z) * ax - sin(pose_rpy.rot_z) * ay) * dt);
    velocity.y += ((sin(pose_rpy.rot_z) * ax + cos(pose_rpy.rot_z) * ay) * dt);
    pose_rpy.x += velocity.x * dt;
    pose_rpy.y += velocity.y * dt;
    last_time = curr_time;
}


void ArdronePoseHandler::odometry(double vx, double vy, double dt)
{
    pose_rpy.x += ((cos(pose_rpy.rot_z) * vx - sin(pose_rpy.rot_z) * vy) * dt) / 1000.0;
    pose_rpy.y += ((sin(pose_rpy.rot_z) * vx + cos(pose_rpy.rot_z) * vy) * dt) / 1000.0;
}
