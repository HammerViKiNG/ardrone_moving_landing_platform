#include "filtered_pose.h"
#include <ros/ros.h>


FilteredPose::FilteredPose(size_t window)
{
    for (size_t i = 0; i < 6; i++)
        filters[i] = new MAFilter(window);
}


void FilteredPose::filter_pose(const PoseRPY& pose)
{
    //ROS_INFO("%f %f %f %f", pose.x, pose.y, pose.z, pose.rot_z);
    filtered_pose.x = filters[0]->get_filtered_value(pose.x);
    filtered_pose.y = filters[1]->get_filtered_value(pose.y);
    filtered_pose.z = filters[2]->get_filtered_value(pose.z);
    filtered_pose.rot_x = filters[3]->get_filtered_value(pose.rot_x);
    filtered_pose.rot_y = filters[4]->get_filtered_value(pose.rot_y);
    filtered_pose.rot_z = filters[5]->get_filtered_value(pose.rot_z);
}


PoseRPY FilteredPose::get_filtered_pose(void)
{
    return filtered_pose;
}
