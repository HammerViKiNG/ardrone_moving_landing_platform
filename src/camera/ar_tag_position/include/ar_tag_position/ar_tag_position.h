#ifndef AR_TAG_POSITION_H
#define AR_TAG_POSITION_H

#include "ros/ros.h"
#include "pose_rpy/pose_rpy.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"

class ArTagPose
{
    public:
        ArTagPose(void) {}
        ArTagPose(ros::NodeHandle nh, std::string bottom_camera_topic, std::string front_camera_topic);
        PoseRPY get_pose(void) const {return pose;}
        bool get_bottom_selected(void) const {return bottom_selected;}
        bool get_is_spotted(void) const {return is_spotted;}
        double get_last_time(void) const {return last_time;}

        void select_bottom_camera(void);
        void select_front_camera(void);

    private:
        ros::Subscriber sub_ar_tag_bottom, sub_ar_tag_front;

        void ar_tag_bottom_callback(const ar_track_alvar_msgs::AlvarMarkers& msg);
        void ar_tag_front_callback(const ar_track_alvar_msgs::AlvarMarkers& msg);

        const int LARGE_AR_TAG_ID = 4;
        const int SMALL_AR_TAG_ID = 8;

        double last_time;

        PoseRPY pose;
        bool bottom_selected;
        bool is_spotted;
};


#endif
