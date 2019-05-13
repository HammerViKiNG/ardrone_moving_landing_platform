#include "ros/ros.h"
#include "ardrone_ar_tag_control.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ar_tag_handler");
    ArdroneARTag ardrone("/ardrone/navdata", "/cmd_vel", "/ardrone/ar_tag_front", "/ardrone/ar_tag_bottom", "gui_control", 200);
    ros::NodeHandle _nh;
    ros::Rate rate(200);
    while (ros::ok())
    {
        ardrone.control();
        ros::spinOnce();
        rate.sleep();
    }
}
