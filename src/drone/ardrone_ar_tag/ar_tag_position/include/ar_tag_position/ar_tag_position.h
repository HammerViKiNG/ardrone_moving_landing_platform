#ifndef AR_TAG_POSITION_H
#define AR_TAG_POSITION_H

#define AR_TAGS_RATIO 9.0

#include "ros/ros.h"
#include <mutex>
#include "pose_rpy/pose_rpy.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "sensor_msgs/CameraInfo.h"

class ArTagPose
{
    public:
        ArTagPose(void) {}
        ArTagPose(std::string bottom_camera_topic, std::string front_camera_topic);

        PoseRPY get_pose(void) const {return pose;}
        bool get_bottom_selected(void) const {return bottom_selected;}
        bool get_is_spotted(void) const {return is_spotted;}
        double get_last_time(void) const {return last_time;}

        void select_bottom_camera(void) {system("rosservice call ardrone/setcamchannel 1");}
        void select_front_camera(void) {system("rosservice call ardrone/setcamchannel 0");}

    private:
        ros::NodeHandle nh;
        ros::Subscriber sub_ar_tag_bottom, sub_ar_tag_front;

        std::mutex mutex;

        void ar_tag_bottom_callback(const ar_track_alvar_msgs::AlvarMarkers& msg);
        void ar_tag_front_callback(const ar_track_alvar_msgs::AlvarMarkers& msg);
        void camera_info_callback(const sensor_msgs::CameraInfo& msg) {bottom_selected = msg.header.frame_id == "ardrone_base_bottomcam";}

        const int LARGE_AR_TAG_ID = 4;
        const int SMALL_AR_TAG_ID = 8;

        double last_time;

        PoseRPY pose;
        bool bottom_selected;
        bool is_spotted;
};


#endif
