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
	    void ar_tag_bottom_callback(const ar_track_alvar_msgs::AlvarMarkers& msg);
        void ar_tag_front_callback(const ar_track_alvar_msgs::AlvarMarkers& msg);

        void correct_necessary_pose_shift(void);
        void stabilize_necessary_pose_shift(void);

        void ar_tag_lost(void);

        int8_t state;
        PoseRPY current_pose, last_pose;
        double z, angular_coords[3];
        double dt;  

        bool is_spotted_bottom, is_spotted_front; 

	PoseRPY necessary_pose_shift;
        
        ros::NodeHandle nh;
        ros::Subscriber sub_tf, sub_navdata, sub_ar_tag_bottom, sub_ar_tag_front;
        ros::Publisher pub_twist;

        tf::Quaternion quat; 
        geometry_msgs::Twist twist;
    
        ros::Time current_time, last_spotted_time;

        ArdronePID* controller, *controller_chasing, *controller_landing;
        ArdronePoseHandler* pose_handler;
};

#endif
