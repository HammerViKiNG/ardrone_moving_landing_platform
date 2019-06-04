#include "ar_tag_position/ar_tag_position.h"


ArTagPose::ArTagPose(ros::NodeHandle nh, std::string bottom_camera_topic, std::string front_camera_topic)
{
    last_time = (ros::Time::now()).toNSec() / 1000000000.0;
    sub_ar_tag_bottom = nh.subscribe("/ardrone/ar_tag_bottom", 1, &ArTagPose::ar_tag_bottom_callback, this);
    sub_ar_tag_front = nh.subscribe("/ardrone/ar_tag_front", 1, &ArTagPose::ar_tag_bottom_callback, this);
}


void ArTagPose::select_bottom_camera(void) 
{
    system("ardrone/setcamchannel 1");
    bottom_selected = true;
}


void ArTagPose::select_front_camera(void) 
{
    system("ardrone/setcamchannel 0");
    bottom_selected = false;
}


void ArTagPose::ar_tag_bottom_callback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
    if (!msg.markers.empty())
    {
        size_t index = 0;
		is_spotted = true;
        if (msg.markers.size() == 2 && msg.markers[1].id == LARGE_AR_TAG_ID)
            index = 1;
        pose = PoseRPY::get_pose_rpy(msg.markers[index].pose.pose);
        pose.rot_z = (pose.rot_z < -M_PI / 2.0) ? 1.5 * M_PI + pose.rot_z : pose.rot_z - M_PI / 2.0;
        
        if (msg.markers[index].id == SMALL_AR_TAG_ID)
        {
	        pose.x /= 9.0;
	        pose.y /= 9.0;
	        pose.z /= 9.0;
        }
        last_time = (ros::Time::now()).toNSec() / 1000000000.0;
    }
}


void ArTagPose::ar_tag_front_callback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
    if (!msg.markers.empty())
    {
        size_t index = 0;
		is_spotted = true;
        if (msg.markers.size() == 2 && msg.markers[1].id == LARGE_AR_TAG_ID)
            index = 1;
	    pose = PoseRPY::get_pose_rpy(msg.markers[index].pose.pose);
        last_time = (ros::Time::now()).toNSec() / 1000000000.0;
    }
}
