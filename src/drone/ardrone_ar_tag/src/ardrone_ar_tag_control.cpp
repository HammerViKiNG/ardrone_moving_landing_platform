#include <cmath>

#include "ros/ros.h"
#include "tf/tfMessage.h"
#include "ardrone_autonomy/Navdata.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

#define limit(value, min, max) (value < min) ? min : (value > max) ? max : value
#define handle_angle(angle) (angle >= 0) ? angle * M_PI / 180.0 : M_PI * (2 - angle / 180.0) 
#define sign(a) (a > 0) - (a < 0)


void tf_callback(const boost::shared_ptr<tf::tfMessage const>& msg, double* linear_coords)
{
    linear_coords[0] = msg->transforms[0].transform.translation.x;
    linear_coords[1] = msg->transforms[0].transform.translation.y; 
    linear_coords[2] = msg->transforms[1].transform.translation.z;
}


void navdata_callback(const boost::shared_ptr<ardrone_autonomy::Navdata const>& msg, double* angular_coords)
{
    angular_coords[0] = msg->rotX * M_PI / 180.0;
    angular_coords[1] = msg->rotY * M_PI / 180.0;
    angular_coords[2] = msg->rotZ * M_PI / 180.0;
}


void ar_tag_callback(const boost::shared_ptr<ar_track_alvar_msgs::AlvarMarkers const>& msg, double* ar_tag_coords, double* angular_coords, double* real_ar_tag_coords, double* linear_coords)
{
    if (msg->markers.size())
    {
        ar_tag_coords[0] = msg->markers[0].pose.pose.position.x;
        ar_tag_coords[1] = msg->markers[0].pose.pose.position.y;
        ar_tag_coords[2] = msg->markers[0].pose.pose.position.z;
        real_ar_tag_coords[0] = linear_coords[2] * tan(atan(ar_tag_coords[0] / ar_tag_coords[2]) - angular_coords[0]);
        real_ar_tag_coords[1] = linear_coords[2] * tan(-atan(ar_tag_coords[1] / ar_tag_coords[2]) - angular_coords[0]);
        ROS_INFO_STREAM(linear_coords[2]);
    }
}


int main(int argc, char** argv)
{
    double linear_coords[3] = {0, 0, 0}, angular_coords[3] = {0, 0, 0}, ar_tag_coords[3] = {0, 0, 0}, real_ar_tag_coords[2] = {0.0, 0.0};
    double x, y;
    ros::init(argc, argv, "ar_tag_handler");
    ros::NodeHandle _nh;
    ros::Subscriber _sub_tf = _nh.subscribe<tf::tfMessage>("/tf", 1, 
        boost::bind(tf_callback, _1, linear_coords));
    ros::Subscriber _sub_navdata = _nh.subscribe<ardrone_autonomy::Navdata>("/ardrone/navdata", 1, 
        boost::bind(navdata_callback, _1, angular_coords));
    ros::Subscriber _sub_ar_tag = _nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ardrone/ar_tag_bottom", 1, 
        boost::bind(ar_tag_callback, _1, ar_tag_coords, angular_coords, real_ar_tag_coords, linear_coords));
    while (ros::ok())
    {
        //ROS_INFO("tag a: %f, b: %f \n drone a: %f, b: %f \n real x: %f, y: %f", atan(ar_tag_coords[0] / ar_tag_coords[2]), atan(-ar_tag_coords[1] / ar_tag_coords[2]), angular_coords[0], angular_coords[1], real_ar_tag_coords[0], real_ar_tag_coords[1]);
        ros::spinOnce();
    }
}
