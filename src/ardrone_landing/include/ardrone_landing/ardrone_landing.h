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

#include "pose_rpy/pose_rpy.h"
#include "ardrone_pose_handler/ardrone_pose_handler.h"
#include "ardrone_pid/ardrone_pid.h"
#include "filtered_pose/filtered_pose.h"
#include "ar_tag_position/ar_tag_position.h"

class ArdroneARTag
{
    public:
        ArdroneARTag(std::string navdata_topic, std::string cmd_topic, std::string bottom_ar_tag_topic, std::string front_ar_tag_topic, std::string gui_control_topic, double hz);
        ~ArdroneARTag();

        void set_necessary_height(float necessary_height) { this->necessary_height = necessary_height; }
        void set_is_hovering(bool is_hovering) { this->is_hovering = is_hovering; }

        void control(void);
    
    private:
        void operate_ar_tag_position(void);

        void correct_necessary_pose_shift(void);
        void stabilize_necessary_pose_shift(void);
        void get_velocity(void);
        void get_camera_object_velocity(void);

        void ar_tag_lost(void);
        void ar_tag_search(void);

        int8_t state;

        bool is_hovering;
        float necessary_height;

        FilteredPose* current_pose_filter;
        PoseRPY current_pose, last_pose;
        double z, angular_coords[3];
        double dt;  

        bool is_spotted_bottom, is_spotted_front; 

        PoseRPY necessary_pose_shift;
        FilteredPose *necessary_pose_shift_global, *necessary_pose_filter, *velocity_filter, *camera_velocity_filter;

        PoseRPY current_spotted_pose, last_spotted_pose, last_necessary_shift, velocity;
        PoseRPY current_camera_object_distance, last_camera_object_distance;
        PoseRPY camera_distance_velocity;
        
        //ros::NodeHandle nh;
        ros::Subscriber sub_tf, sub_navdata, sub_ar_tag, sub_gui_control;
        ros::Publisher pub_twist;

        tf::Quaternion quat; 
        geometry_msgs::Twist twist;
    
        ros::Time last_time, last_spotted_time, current_spotted_time;

        ArdronePID *controller, *controller_chasing, *controller_landing;
        ArdronePoseHandler* pose_handler;

        ArTagPose ar_tag_position;
};

#endif
