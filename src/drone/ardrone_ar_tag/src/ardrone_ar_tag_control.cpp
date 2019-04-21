#include "ardrone_ar_tag_control.h"


ArdroneARTag::ArdroneARTag(std::string navdata_topic, std::string cmd_topic, std::string ar_tag_front_topic, std::string ar_tag_bottom_topic, double hz)
{
    sub_ar_tag_bottom = nh.subscribe(ar_tag_bottom_topic, 1, &ArdroneARTag::ar_tag_bottom_callback, this);
    sub_ar_tag_front = nh.subscribe(ar_tag_front_topic, 1, &ArdroneARTag::ar_tag_front_callback, this);
    pub_twist = nh.advertise<geometry_msgs::Twist>(cmd_topic, 1);

    dt = 1 / hz;

    is_spotted_bottom = is_spotted_front = false;

    current_pose_filter = new FilteredPose(2);
    necessary_pose_shift_global = new FilteredPose(2);
    necessary_pose_filter = new FilteredPose(2);
    velocity_filter = new FilteredPose(2);

    pose_handler = new ArdronePoseHandler(navdata_topic);
    current_pose_filter->filter_pose(pose_handler->get_pose_rpy());
    current_pose = current_pose_filter->get_filtered_pose();

    velocity = PoseRPY::make_pose_rpy();

    double* k_p = new double[4] {10, 0.5, 5, 5};
    double* k_d = new double[4] {0, 0.3, 2, 2.5};
    double* k_i = new double[4] {2, 0.0, 0.5, 0.0};
    double* crit = new double[4] {3, 3, 2, 1};
    double* max_int_rel = new double[6] {2, 0.0, 0.5, 0.0};
    controller_chasing = new ArdronePID(hz, k_p, k_d, k_i, crit, max_int_rel);
    
    k_p = new double[4] {0.7, 0.7, 5, 4};
    k_d = new double[4] {0.3, 0.3, 2, 2};
    k_i = new double[4] {0.0, 0.0, 0, 0.0};
    crit = new double[4] {3, 3, 2, 1};
    max_int_rel = new double[6] {0.0, 0.0, 0.0, 0.0};
    controller_landing = new ArdronePID(hz, k_p, k_d, k_i, crit, max_int_rel);
    delete k_p, k_d, k_i, crit, max_int_rel;
}


void ArdroneARTag::correct_necessary_pose_shift(void)
{
    current_pose_filter->filter_pose(pose_handler->get_pose_rpy());
    current_pose = current_pose_filter->get_filtered_pose();
    PoseRPY delta_pose = current_pose - last_pose;
    PoseRPY necessary_shift_global = ArdronePoseHandler::local_to_global_shifted(necessary_pose_shift, current_pose.rot_z) - delta_pose;
    if (!is_spotted_front)
    {
        necessary_shift_global.x += 0.1 * velocity.x * dt;
        necessary_shift_global.y += 0.1 * velocity.y * dt;
    }
    necessary_pose_shift_global->filter_pose(necessary_shift_global);
    necessary_pose_shift = ArdronePoseHandler::global_to_local_shifted(necessary_pose_shift_global->get_filtered_pose(), current_pose.rot_z);
    last_pose = current_pose;
    //ROS_INFO("necessary_pose x: %f, y: %f, z: %f, rot_z: %f", necessary_pose_shift.x, necessary_pose_shift.y, necessary_pose_shift.z, necessary_pose_shift.rot_z);
}


void ArdroneARTag::stabilize_necessary_pose_shift(void)
{
    //double d_rot_x = necessary_pose_shift.rot_x - current_pose.rot_x,
    //       d_rot_y = necessary_pose_shift.rot_y - current_pose.rot_y,
    //       d_rot_z = necessary_pose_shift.rot_z - current_pose.rot_z;
    //double d_rot_x = necessary_pose_shift.rot_x,
    //       d_rot_y = necessary_pose_shift.rot_y,
    //       d_rot_z = necessary_pose_shift.rot_z;
    //necessary_pose_shift.z /= cos(d_rot_x) * cos(d_rot_y);
    //necessary_pose_shift.x -= necessary_pose_shift.z * tan(d_rot_x);
    //necessary_pose_shift.y += necessary_pose_shift.z * tan(d_rot_y);
    necessary_pose_shift = PoseRPY::transform_pose_3d(necessary_pose_shift, -current_pose.rot_x, -current_pose.rot_y, 0);
}


void ArdroneARTag::get_velocity(void)
{
    PoseRPY delta_necessary_shift = ArdronePoseHandler::local_to_global_shifted(necessary_pose_shift, current_pose.rot_z) - ArdronePoseHandler::local_to_global_shifted(last_necessary_shift, last_spotted_pose.rot_z),
    delta_pose = current_pose - last_spotted_pose;
    velocity_filter->filter_pose((delta_necessary_shift - delta_pose) *  1000000000.0 / (ros::Time::now() - last_spotted_time).toNSec());
    velocity = velocity_filter->get_filtered_pose();
    ROS_INFO("vel x: %f, y: %f", velocity.x, velocity.y);
}


void ArdroneARTag::ar_tag_lost(void)
{
    current_pose_filter->filter_pose(pose_handler->get_pose_rpy());
    current_pose = current_pose_filter->get_filtered_pose();
    necessary_pose_shift.z = 2.5 - current_pose.z;
}


void ArdroneARTag::ar_tag_search(void)
{
    current_pose_filter->filter_pose(pose_handler->get_pose_rpy());
    current_pose = current_pose_filter->get_filtered_pose();
    necessary_pose_shift.x = necessary_pose_shift.y = 0;
    necessary_pose_shift.z = current_pose.z - 2.5;
    necessary_pose_shift.rot_z = M_PI / 5;
}


void ArdroneARTag::ar_tag_bottom_callback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
    size_t index = 0;
    if (!msg.markers.empty())
    {
        if (msg.markers.size() == 2 && msg.markers[1].id == 4)
            index = 1;
	    tf::quaternionMsgToTF(msg.markers[index].pose.pose.orientation, quat);
    	tf::Matrix3x3(quat).getRPY(necessary_pose_shift.rot_x, necessary_pose_shift.rot_y, necessary_pose_shift.rot_z);
        necessary_pose_shift.rot_z = (necessary_pose_shift.rot_z < -M_PI / 2.0) ? 1.5 * M_PI + necessary_pose_shift.rot_z : necessary_pose_shift.rot_z - M_PI / 2.0;
        
        if (msg.markers[index].id == 8)
        {
	        necessary_pose_shift.x = msg.markers[index].pose.pose.position.x / 9.0;
            necessary_pose_shift.y = msg.markers[index].pose.pose.position.y / 9.0;
            necessary_pose_shift.z = msg.markers[index].pose.pose.position.z / 9.0;
        }
        else
        {
            necessary_pose_shift.x = msg.markers[index].pose.pose.position.x;
            necessary_pose_shift.y = msg.markers[index].pose.pose.position.y;
            necessary_pose_shift.z = msg.markers[index].pose.pose.position.z;
        }

        if (is_spotted_front || is_spotted_bottom)
            get_velocity(); 

        is_spotted_bottom = true;
        is_spotted_front = false;

        stabilize_necessary_pose_shift();
        //necessary_pose_filter->filter_pose(necessary_pose_shift);
        //necessary_pose_shift = necessary_pose_filter->get_filtered_pose();      

        //ROS_INFO("x: %f, y: %f, z: %f, rot_x: %f, rot_y: %f", necessary_pose_shift.x, necessary_pose_shift.y, necessary_pose_shift.z, necessary_pose_shift.rot_x, necessary_pose_shift.rot_y);

        last_spotted_pose = current_pose;
        last_necessary_shift = necessary_pose_shift;

        last_pose = current_pose;
        last_spotted_time = ros::Time::now();
    }
}


void ArdroneARTag::ar_tag_front_callback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
    if (!msg.markers.empty())
    {
        size_t index = 0;
        if (msg.markers.size() == 2 && msg.markers[1].id == 4)
            index = 1;
	    tf::quaternionMsgToTF(msg.markers[index].pose.pose.orientation, quat);
        tf::Matrix3x3(quat).getRPY(necessary_pose_shift.rot_x, necessary_pose_shift.rot_y, necessary_pose_shift.rot_z);
        necessary_pose_shift.x = msg.markers[index].pose.pose.position.x;
        necessary_pose_shift.y = msg.markers[index].pose.pose.position.y;
        necessary_pose_shift.z = 2.5 - msg.markers[index].pose.pose.position.z;
        necessary_pose_shift.rot_z = atan(msg.markers[index].pose.pose.position.y / necessary_pose_shift.x);

        stabilize_necessary_pose_shift();

        //necessary_pose_filter->filter_pose(necessary_pose_shift);
        //necessary_pose_shift = necessary_pose_filter->get_filtered_pose();
        if (is_spotted_front || is_spotted_bottom)
            get_velocity();  
        is_spotted_front = true;
        is_spotted_bottom = false;
        last_spotted_pose = current_pose;
        last_necessary_shift = necessary_pose_shift;
        last_pose = current_pose;
        last_spotted_time = ros::Time::now();
    }
}


void ArdroneARTag::control(void)
{
    if (state == 2)
    {
        //system("rostopic pub -1 /ardrone/takeoff std_msgs/Empty");
    }
    else if (is_spotted_bottom || is_spotted_front)
    {
        //ROS_INFO("necessary shift: x: %f, y: %f, z: %f, yaw: %f, state: %d", necessary_pose_shift.x, necessary_pose_shift.y, necessary_pose_shift.z, necessary_pose_shift.rot_z, pose_handler->get_state());
        correct_necessary_pose_shift();
        //ROS_INFO("x: %f, y: %f", current_pose.rot_x, current_pose.rot_y);
        controller = (is_spotted_bottom) ? controller_landing : controller_chasing;
        if ((ros::Time::now() - last_spotted_time).toNSec() / 1000000000.0 >= 1.0)
            ar_tag_search();
        else if ((ros::Time::now() - last_spotted_time).toNSec() / 1000000000.0 >= 0.25)
            ar_tag_lost();
        else if (pose_handler->get_state()!= 8 && pose_handler->get_state()!= 2 && current_pose.z <= 0.2 && is_spotted_bottom)
	    {
            system("rostopic pub -1 /ardrone/land std_msgs/Empty");
            is_spotted_bottom = is_spotted_front = 0;
        }
        else
        {
            //twist = controller->pid_twist(necessary_pose_shift);
            //ROS_INFO("%f, %f, %f, %f", twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z);
            //pub_twist.publish(twist);
        }
    }
    else 
    {
    }
}


