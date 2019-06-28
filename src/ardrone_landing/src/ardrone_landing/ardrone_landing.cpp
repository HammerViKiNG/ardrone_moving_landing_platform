#include "ardrone_landing/ardrone_landing.h"
#define limit(value, min, max) (value < min) ? min : (value > max) ? max : value


ArdroneARTag::ArdroneARTag(std::string navdata_topic, std::string cmd_topic, std::string bottom_ar_tag_topic, std::string front_ar_tag_topic, std::string gui_control_topic, double hz):
  ar_tag_position(bottom_ar_tag_topic, front_ar_tag_topic)
{
    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

    pub_twist = nh->advertise<geometry_msgs::Twist>(cmd_topic, 1);

    dt = 1 / hz;

    is_spotted_bottom = is_spotted_front = false;

    current_pose_filter = new FilteredPose(2);
    necessary_pose_shift_global = new FilteredPose(2);
    necessary_pose_filter = new FilteredPose(2);
    velocity_filter = new FilteredPose(2);

    pose_handler = new ArdronePoseHandler(navdata_topic);
    current_pose_filter->filter_pose(pose_handler->get_pose_rpy());
    current_pose = current_pose_filter->get_filtered_pose();

    velocity = PoseRPY::zero_pose_rpy();

    /*double* k_p = new double[4] {10, 0.5, 5, 5};
    double* k_d = new double[4] {0, 0.3, 2, 1};
    double* k_i = new double[4] {2, 0.0, 0.5, 0.0};
    double* crit = new double[4] {0.5, 0.5, 0.25, 0.5};
    double* max_int_rel = new double[6] {0.5, 0.0, 0.25, 0.0};
    controller_chasing = new ArdronePID(hz, k_p, k_d, k_i, crit, max_int_rel);
    
    k_p = new double[4] {5, 2, 5, 5};
    k_d = new double[4] {0.5, 0.5, 2, 1};
    k_i = new double[4] {0.25, 0.0, 0, 0.25};
    crit = new double[4] {0.5, 0.5, 0.25, 0.5};
    max_int_rel = new double[6] {0.25, 0.0, 0.0, 0.25};
    controller_landing = new ArdronePID(hz, k_p, k_d, k_i, crit, max_int_rel);
    delete k_p, k_d, k_i, crit, max_int_rel;*/

    double* k_p = new double[4] {10, 0.5, 5, 5};
    double* k_d = new double[4] {0, 0.3, 2, 1};
    double* k_i = new double[4] {2, 0.0, 0.5, 0.0};
    double* crit = new double[4] {0.5, 0.5, 0.25, 0.5};
    double* max_int_rel = new double[6] {0.5, 0.0, 0.25, 0.0};
    controller_chasing = new ArdronePID(hz, k_p, k_d, k_i, crit, max_int_rel);
    
    k_p = new double[4] {0.25, 0.25, 0.1, 0.5};
    k_d = new double[4] {0.0, 0.0, 0, 0};
    k_i = new double[4] {0.0, 0.0, 0, 0.0};
    crit = new double[4] {0.15, 0.15, 0.15, 0.15};
    max_int_rel = new double[6] {0.0, 0.0, 0.0, 0.0};
    controller_landing = new ArdronePID(hz, k_p, k_d, k_i, crit, max_int_rel);
    delete k_p, k_d, k_i, crit, max_int_rel;

    is_hovering = false;
}


ArdroneARTag::~ArdroneARTag()
{
    sub_ar_tag.shutdown();
    pub_twist.shutdown();
}


void ArdroneARTag::correct_necessary_pose_shift(void)
{
    current_pose_filter->filter_pose(pose_handler->get_pose_rpy());
    current_pose = current_pose_filter->get_filtered_pose();
    PoseRPY delta_pose = current_pose - last_pose;
    PoseRPY necessary_shift_global = ArdronePoseHandler::local_to_global_shifted(necessary_pose_shift, current_pose.rot_z) - delta_pose;
    velocity = pose_handler->get_velocity();
    if (!is_spotted_front && abs(velocity.x) <= 2.0 && abs(velocity.y) <= 2.0)
    {
        necessary_shift_global.x += limit(1.0 * velocity.x * dt, -1, 1);
        necessary_shift_global.y += limit(1.0 * velocity.y * dt, -1, 1);
    }
    necessary_pose_shift_global->filter_pose(necessary_shift_global);
    necessary_pose_shift = ArdronePoseHandler::global_to_local_shifted(necessary_pose_shift_global->get_filtered_pose(), current_pose.rot_z);
    last_pose = current_pose;
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
    necessary_pose_shift = PoseRPY::transform_pose_3d(necessary_pose_shift, -current_pose.rot_x, current_pose.rot_y, 0);
}


void ArdroneARTag::get_velocity(void)
{
    current_pose_filter->filter_pose(pose_handler->get_pose_rpy());
    current_pose = current_pose_filter->get_filtered_pose();
    PoseRPY delta_necessary_shift = ArdronePoseHandler::local_to_global_shifted(necessary_pose_shift, current_pose.rot_z) - ArdronePoseHandler::local_to_global_shifted(last_necessary_shift, last_spotted_pose.rot_z),
    delta_pose = current_pose - last_spotted_pose;
    velocity_filter->filter_pose((delta_necessary_shift + delta_pose) * (ros::Time::now().toNSec() / 1e+9 - last_spotted_time));
    velocity = velocity_filter->get_filtered_pose();
    //ROS_INFO("vel x: %f, y: %f", velocity.x, velocity.y);
}


void ArdroneARTag::ar_tag_lost(void)
{
    current_pose_filter->filter_pose(pose_handler->get_pose_rpy());
    current_pose = current_pose_filter->get_filtered_pose();
    necessary_pose_shift.z = 1.5 - current_pose.z;
}


void ArdroneARTag::ar_tag_search(void)
{
    current_pose_filter->filter_pose(pose_handler->get_pose_rpy());
    current_pose = current_pose_filter->get_filtered_pose();
    necessary_pose_shift.x = necessary_pose_shift.y = 0;
    necessary_pose_shift.z = 1.5 - current_pose.z;
    necessary_pose_shift.rot_z = M_PI / 5;
}


void ArdroneARTag::operate_ar_tag_position(void)
{
    if (ar_tag_position.get_last_time() != last_spotted_time)
    {
        necessary_pose_shift = ar_tag_position.get_pose();
        //ROS_INFO("%f, %f, %f, %f", necessary_pose_shift.x, necessary_pose_shift.y, necessary_pose_shift.z, necessary_pose_shift.rot_z);
        //ROS_INFO("%f, %f, %f, %f", ar_tag_position.get_pose().x, ar_tag_position.get_pose().y, ar_tag_position.get_pose().z, ar_tag_position.get_pose().rot_z);

        stabilize_necessary_pose_shift();

        is_spotted_bottom = ar_tag_position.get_bottom_selected();
        is_spotted_front = !is_spotted_bottom;
        
        last_spotted_pose = current_pose;
        last_necessary_shift = necessary_pose_shift;

        last_pose = current_pose;
        last_spotted_time = ros::Time::now().toNSec() / 1e+9;
    }
}


void ArdroneARTag::control(void)
{
    current_time = ros::Time::now().toNSec() / 1e+9;
    operate_ar_tag_position();
    ROS_INFO("%f, %f, %f, %f", pose_handler->get_pose_rpy().x, pose_handler->get_pose_rpy().y, pose_handler->get_pose_rpy().z, pose_handler->get_pose_rpy().rot_z);
    if (state == 2)
    {
        //system("rostopic pub -1 /ardrone/takeoff std_msgs/Empty");
    }
    else if (is_spotted_bottom || is_spotted_front)
    {
        //ROS_INFO("necessary shift: x: %f, y: %f, z: %f, yaw: %f, state: %d", necessary_pose_shift.x, necessary_pose_shift.y, necessary_pose_shift.z, necessary_pose_shift.rot_z, pose_handler->get_state());
        correct_necessary_pose_shift();
        controller = (is_spotted_bottom) ? controller_landing : controller_chasing;
        if (pose_handler->get_state()!= 8 && pose_handler->get_state()!= 2 && current_pose.z <= 0.2 && is_spotted_bottom)
        {
            system("rostopic pub -1 /ardrone/land std_msgs/Empty");
            is_spotted_bottom = is_spotted_front = 0;
        }
        if (current_time - last_spotted_time >= 1.0)
            ar_tag_search();
        else if (current_time - last_spotted_time >= 0.25)
            ar_tag_lost();
        else
        {
            twist = controller->pid_twist(necessary_pose_shift);
            pub_twist.publish(twist);
        }
    }
    else 
    {
    }
}


