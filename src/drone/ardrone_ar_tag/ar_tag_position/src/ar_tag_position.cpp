#include "ar_tag_position/ar_tag_position.h"


ArTagPose::ArTagPose(std::string bottom_camera_topic, std::string front_camera_topic)
{
    //select_bottom_camera();
    select_bottom_camera();
    last_time = ros::Time::now();
    sub_ar_tag_bottom = nh.subscribe("/ardrone/ar_tag_bottom", 1, &ArTagPose::ar_tag_bottom_callback, this);
    sub_ar_tag_front = nh.subscribe("/ardrone/ar_tag_front", 1, &ArTagPose::ar_tag_front_callback, this);
}


void ArTagPose::select_bottom_camera(void) {
    system("rosservice call ardrone/setcamchannel 1");
    bottom_selected = true;
}


void ArTagPose::select_front_camera(void) {
    system("rosservice call ardrone/setcamchannel 0");
    bottom_selected = false;
}


void ArTagPose::ar_tag_bottom_callback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
    if (!msg.markers.empty())
    {
        std::lock_guard<std::mutex> lock(mutex);
        size_t index = 0;
        is_spotted_bottom = true;
        is_spotted_front = false;
        if (msg.markers.size() == 2 && msg.markers[1].id == LARGE_AR_TAG_ID)
            index = 1;
        pose = PoseRPY::get_pose_rpy(msg.markers[index].pose.pose);
        pose.rot_z = (pose.rot_z < -M_PI / 2.0) ? 1.5 * M_PI + pose.rot_z : pose.rot_z - M_PI / 2.0;
        //real pose.x *= -1;
        //real pose.y *= -1;
        
        if (msg.markers[index].id == SMALL_AR_TAG_ID)
        {
	        pose.x /= AR_TAGS_RATIO;
	        pose.y /= AR_TAGS_RATIO;
	        pose.z /= AR_TAGS_RATIO;
        }

        last_time = ros::Time::now();
        bottom_selected = true;
    }
}


void ArTagPose::ar_tag_front_callback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
    if (!msg.markers.empty())
    {
        std::lock_guard<std::mutex> lock(mutex);
        size_t index = 0;
        is_spotted_front = true;
        is_spotted_bottom = false;
        if (msg.markers.size() == 2 && msg.markers[1].id == LARGE_AR_TAG_ID)
            index = 1;
        pose = PoseRPY::get_pose_rpy(msg.markers[index].pose.pose);
        //real pose.y *= -1;
        pose.rot_z = atan(pose.y / pose.x);

        if (msg.markers[index].id == SMALL_AR_TAG_ID)
        {
	        pose.x /= AR_TAGS_RATIO;
	        pose.y /= AR_TAGS_RATIO;
	        pose.z /= AR_TAGS_RATIO;
        }

        last_time = ros::Time::now();
        bottom_selected = false;
    }
}
