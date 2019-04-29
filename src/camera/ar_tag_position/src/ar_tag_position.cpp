#include "ros/ros.h"

#include "pose_rpy/pose_rpy.h"
#include "pose_rpy/PoseRPY.h"

#include "ar_tag_position/ArTagPose.h"

#include "ar_track_alvar_msgs/AlvarMarkers.h"

const int LARGE_AR_TAG_ID = 4;
const int SMALL_AR_TAG_ID = 8;

double last_time;
bool is_spotted;

PoseRPY pose;
ar_tag_position::ArTagPose ar_tag_pose;


void ar_tag_bottom_callback(const ar_track_alvar_msgs::AlvarMarkers& msg)
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
        ar_tag_pose.pose = pose;
        ar_tag_pose.camera_name = "bottom";
		ar_tag_pose.is_spotted = is_spotted;
        last_time = (ros::Time::now()).toNSec() / 1000000000.0;
    }
}


void ar_tag_front_callback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
    if (!msg.markers.empty())
    {
        size_t index = 0;
		is_spotted = true;
        if (msg.markers.size() == 2 && msg.markers[1].id == LARGE_AR_TAG_ID)
            index = 1;
	    pose = PoseRPY::get_pose_rpy(msg.markers[index].pose.pose);
        ar_tag_pose.pose = pose;
        ar_tag_pose.camera_name = "front";
		ar_tag_pose.is_spotted = is_spotted;
        last_time = (ros::Time::now()).toNSec() / 1000000000.0;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ar_tag_position");
    ros::NodeHandle nh;
	last_time = (ros::Time::now()).toNSec() / 1000000000.0;
    std::string bottom_camera_topic, front_camera_topic, output_topic;
    /*if (nh.getParam("bottom_camera_topic", bottom_camera_topic))
        ros::Subscriber sub_ar_tag_bottom = nh.subscribe(bottom_camera_topic, 1, ar_tag_bottom_callback);
    if (nh.getParam("front_camera_topic", front_camera_topic))
        ros::Subscriber sub_ar_tag_front = nh.subscribe(front_camera_topic, 1, ar_tag_bottom_callback);
    if (nh.getParam("output_topic", output_topic))
        pub_pose = nh.advertise<ar_tag_position::ArTagPose>(output_topic, 1);*/
    ros::Subscriber sub_ar_tag_bottom = nh.subscribe("/ardrone/ar_tag_bottom", 1, ar_tag_bottom_callback);
    ros::Subscriber sub_ar_tag_front = nh.subscribe("/ardrone/ar_tag_front", 1, ar_tag_bottom_callback);
    ros::Publisher pub_pose = nh.advertise<ar_tag_position::ArTagPose>("ar_tag_position", 1);
	ros::Rate rate(1000);
    while (ros::ok())
	{
		ar_tag_pose.period = ((ros::Time::now()).toNSec() / 1000000000.0 - last_time);
		pub_pose.publish(ar_tag_pose);
        ros::spinOnce();
		rate.sleep();
	}
    return 0;
}
