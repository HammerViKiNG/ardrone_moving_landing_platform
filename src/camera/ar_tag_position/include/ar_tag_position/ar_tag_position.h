#ifndef AR_TAG_POSITION_H
#define AR_TAG_POSITION_H

#include "ros/ros.h"
#include "pose_rpy/pose_rpy.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

class ArTagPose
{
    public:
        ArTagPose(ros::NodeHandle nh, std::string bottom_camera_topic, std::string front_camera_topic);
        PoseRPY get_pose(void) {return pose;}
        std::string get_camera_name(void) {return camera_name;}
        bool get_is_spotted(void) {return is_spotted;}

    private:
        ros::Subscriber sub_ar_tag_bottom, sub_ar_tag_front;

        void ar_tag_bottom_callback(const ar_track_alvar_msgs::AlvarMarkers& msg);
        void ar_tag_front_callback(const ar_track_alvar_msgs::AlvarMarkers& msg);

        const int LARGE_AR_TAG_ID = 4;
        const int SMALL_AR_TAG_ID = 8;

        double last_time;

        PoseRPY pose;
        std::string camera_name;
        bool is_spotted;
};


#endif
