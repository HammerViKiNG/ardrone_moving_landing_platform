#ifndef ARDRONE_AR_TAG_CONTROL_H
#define ARDRONE_AR_TAG_CONTROL_H

#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include "ardrone_autonomy/Navdata.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "geometry_msgs/Twist.h"

#include "ardrone_pid.h"


class ArdroneARTag
{
    public:
        ArdroneARTag(std::string navdata_topic, std::string cmd_topic, std::string ar_tag_front_topic, std::string ar_tag_bottom_topic, double hz);
        void control(void);
    
    private:
        void navdata_callback(const ardrone_autonomy::Navdata& msg);
	    void ar_tag_bottom_callback(const ar_track_alvar_msgs::AlvarMarkers& msg);

        int8_t state;
        double z, angular_coords[3];
        double dt;  

        bool is_spotted_bottom; 

	    PoseRPY necessary_pose;
        
        ros::NodeHandle nh;
        ros::Subscriber sub_tf, sub_navdata, sub_ar_tag_bottom;
        ros::Publisher pub_twist;

        tf::Quaternion quat; 
        geometry_msgs::Twist twist;


        ArdronePID* controller;
};

#endif
