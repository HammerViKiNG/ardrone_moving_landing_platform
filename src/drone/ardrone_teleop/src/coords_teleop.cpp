#include <cmath>

#include "ros/ros.h"
#include "tf/tfMessage.h"
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"

#define limit(value, min, max) (value < min) ? min : (value > max) ? max : value
#define handle_angle(angle) (angle >= 0) ? angle * M_PI / 180.0 : M_PI * (2 - angle / 180.0) 
#define sign(a) (a > 0) - (a < 0)

const float K_P = 0.5;

void tf_callback(const boost::shared_ptr<tf::tfMessage const>& msg, double* linear_coords)
{
    linear_coords[0] = msg->transforms[0].transform.translation.x;
    linear_coords[1] = msg->transforms[0].transform.translation.y; 
}

void navdata_callback(const boost::shared_ptr<ardrone_autonomy::Navdata const>& msg, double* angular_coords, double* linear_coords)
{
    angular_coords[0] = msg->rotX;
    angular_coords[1] = msg->rotY;
    angular_coords[2] = msg->rotZ;
    linear_coords[2] = msg->altd / 1000.0;
}

void PID(geometry_msgs::Twist& twist, double* necessary_coords, double* linear_coords, double* angular_coords)
{
    float* e = new float[3];
    float* rot = new float[3];
    for (size_t i = 0; i < 3; i++)
    { 
        e[i] = necessary_coords[i] - linear_coords[i];
        rot[i] = handle_angle(angular_coords[i]);
    }
    float* orient = new float[3] { atan(e[1] / e[2]) - rot[1],
                                   atan(e[0] / e[2]) - rot[2],
                                   atan(e[1] / e[0]) - rot[0]};
    float* d_1 = new float[2] {sqrt(pow(e[0], 2) + pow(e[1], 2)) * cos(orient[2]),
                               sqrt(pow(e[0], 2) + pow(e[1], 2)) * sin(orient[2])};
    float* d = new float[3] {sign(e[0]) * sqrt(pow(d_1[0], 2) + pow(e[2], 2)) * sin(orient[0]),
                             sign(e[1]) * sqrt(pow(d_1[1], 2) + pow(e[2], 2)) * sin(orient[1]),
                             (float)sign(e[2]) * sqrt( (pow(d_1[0], 2) + pow(e[2], 2)) * pow(cos(orient[0]), 2) + (pow(d_1[1], 2) + pow(e[2], 2)) * pow(cos(orient[1]), 2) )};
    ROS_INFO("necessary: %f, %f, %f", necessary_coords[0], necessary_coords[1], necessary_coords[2]);
    ROS_INFO("real: %f, %f, %f", linear_coords[0], linear_coords[1], linear_coords[2]);
    twist.linear.x = limit(K_P * d[0], -1, 1);
    twist.linear.y = limit(K_P * d[1], -1, 1);
    twist.linear.z = limit(K_P * d[2], -1, 1);
    delete e, rot, orient, d_1, d;
}


int main(int argc, char** argv)
{
    double necessary_coords[3], linear_coords[3], angular_coords[3];
    ros::init(argc, argv, "coords_teleop");
    ros::NodeHandle _nh;
    ros::Subscriber _sub_tf = _nh.subscribe<tf::tfMessage>("/tf", 1, 
        boost::bind(tf_callback, _1, linear_coords));
    ros::Subscriber _sub_navdata = _nh.subscribe<ardrone_autonomy::Navdata>("/ardrone/navdata", 1, 
        boost::bind(navdata_callback, _1, angular_coords, linear_coords));
    ros::Publisher _pub_twist = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    if (argc == 4)
        for (size_t i = 0; i < 3; i++)
            necessary_coords[i] = atof(argv[i + 1]);
    else
        for (size_t i = 0; i < 3; i++)
            necessary_coords[i] = 0;
    geometry_msgs::Twist twist;
    system("rostopic pub -1 /ardrone/takeoff std_msgs/Empty");
    while (ros::ok())
    {
        PID(twist, necessary_coords, linear_coords, angular_coords);
        _pub_twist.publish(twist);
        ros::spinOnce();
    }
}
