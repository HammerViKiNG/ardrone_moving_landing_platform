#ifndef ARDRONE_AR_TAG_CONTROL_H
#define ARDRONE_AR_TAG_CONTROL_H

#include <cmath>

#include "ros/ros.h"
#include "tf/tfMessage.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"
#include "ardrone_autonomy/Navdata.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "geometry_msgs/Twist.h"

#include "ardrone_pid.h"


class ArdroneARTag
{
    public:
        ArdroneARTag(std::string tf_topic, std::string navdata_topic, std::string cmd_topic, std::string ar_tag_front_topic, std::string ar_tag_bottom_topic, double hz);
        void control(void);
    
    private:
        void tf_callback(const tf::tfMessage& msg);
        void navdata_callback(const ardrone_autonomy::Navdata& msg);
	    void ar_tag_bottom_callback(const ar_track_alvar_msgs::AlvarMarkers& msg);

        int8_t state;
        double necessary_coords[3], linear_coords[3], angular_coords[3], ar_tag_coords[2];
        double dt;  

        bool is_spotted_bottom; 
        
        ros::NodeHandle nh;
        ros::Subscriber sub_tf, sub_navdata, sub_ar_tag_bottom;
        ros::Publisher pub_twist;

        geometry_msgs::Twist twist;
        tf::TransformListener listener;

        ArdronePID* controller;
};

#endif
