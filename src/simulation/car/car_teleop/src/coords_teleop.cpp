#include "ros/ros.h"
#include "ardrone_pid.h"

int main(int argc, char** argv)
{
    double necessary_coords[3];
    ros::init(argc, argv, "coords_teleop");
    ArdronePID pid("/tf", "/ardrone/navdata", "/cmd_vel", 200);
    ros::NodeHandle _nh;
    if (argc == 4)
        for (size_t i = 0; i < 3; i++)
            necessary_coords[i] = atof(argv[i + 1]);
    else
        for (size_t i = 0; i < 3; i++)
            necessary_coords[i] = 0;
    pid.set_necessary_coords(necessary_coords);
    ros::Rate rate(200);
    while (ros::ok())
    {
        pid.control();
        ros::spinOnce();
        rate.sleep();
    }
}
