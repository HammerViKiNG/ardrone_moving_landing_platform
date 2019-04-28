#include "ros/ros.h"

#include "pose_rpy/pose_rpy.h"
#include "pose_rpy/PoseRPY.h"

#include "ar_tag_position/ArTagPose.h"

#include "ar_track_alvar_msgs/AlvarMarkers.h"


const int LARGE_AR_TAG_ID = 4;
const int SMALL_AR_TAG_ID = 8;

ros::NodeHandle nh;
ros::Publisher pub_pose;

PoseRPY pose;
ar_tag_position::ArTagPose ar_tag_pose;


void ar_tag_bottom_callback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
    if (!msg.markers.empty())
    {
        size_t index = 0;
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
        pub_pose.publish(ar_tag_pose);
    }
}


void ar_tag_front_callback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
    if (!msg.markers.empty())
    {
        size_t index = 0;
        if (msg.markers.size() == 2 && msg.markers[1].id == LARGE_AR_TAG_ID)
            index = 1;
	pose = PoseRPY::get_pose_rpy(msg.markers[index].pose.pose);
        ar_tag_pose.pose = pose;
        ar_tag_pose.camera_name = "front";
        pub_pose.publish(ar_tag_pose);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ar_tag_position");
    if (argc == 3)
    {
        ros::Subscriber sub_ar_tag_bottom = nh.subscribe(argv[0], 1, ar_tag_bottom_callback),
                        sub_ar_tag_front = nh.subscribe(argv[1], 1, ar_tag_bottom_callback);
        pub_pose = nh.advertise<ar_tag_position::ArTagPose>(argv[2], 1);
        while (ros::ok())
            ros::spinOnce();
    }
    return 0;
}
