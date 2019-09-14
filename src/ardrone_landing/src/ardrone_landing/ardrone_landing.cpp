#include "ardrone_landing/ardrone_landing.h"
#define limit(value, min, max) (value < min) ? min : (value > max) ? max : value


ArdroneARTag::ArdroneARTag(std::string navdata_topic, std::string cmd_topic, std::string bottom_ar_tag_topic, std::string front_ar_tag_topic, std::string gui_control_topic, double hz):
  ar_tag_position(bottom_ar_tag_topic, front_ar_tag_topic)
{
    size_t filter_capacity = 2;

    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();

    pub_twist = nh->advertise<geometry_msgs::Twist>(cmd_topic, 1);

    dt = 1 / hz;

    is_spotted_bottom = is_spotted_front = false;

    current_pose_filter = new FilteredPose(filter_capacity);
    necessary_pose_shift_global = new FilteredPose(filter_capacity);
    necessary_pose_filter = new FilteredPose(filter_capacity);
    velocity_filter = new FilteredPose(filter_capacity);
    camera_velocity_filter = new FilteredPose(filter_capacity);


    pose_handler = new ArdronePoseHandler(navdata_topic);
    current_pose_filter->filter_pose(pose_handler->get_pose_rpy());
    current_pose = current_pose_filter->get_filtered_pose();

    velocity = PoseRPY::zero_pose_rpy();

    double* k_p = new double[4] {5, 0.5, 5, 5};
    double* k_d = new double[4] {0.0, 0.0, 0, 0};
    double* k_i = new double[4] {0, 0.0, 0.0, 0.0};
    double* crit = new double[4] {1.0, 0.5, 0.0, 0.5};
    double* max_int_rel = new double[4] {0.0, 0.0, 0.25, 0.0};
    controller_chasing = new ArdronePID(k_p, k_d, k_i, crit, max_int_rel);
    
    /*k_p = new double[4] {5, 5, 5, 5};
    k_d = new double[4] {0.25, 0.25, 0.1, 0};
    k_i = new double[4] {0.25, 0.25, 0, 0.25};
    crit = new double[4] {0.5, 0.5, 0.25, 0.5};
    max_int_rel = new double[6] {0.25, 0.25, 0.0, 0.25};
    controller_landing = new ArdronePID(k_p, k_d, k_i, crit, max_int_rel);
    delete[] k_p, k_d, k_i, crit, max_int_rel;*/

    k_p = new double[4] {2.0, 2.0, 2, 2};
    k_d = new double[4] {0.0, 0.0, 0.0, 0.0};
    k_i = new double[4] {0.0, 0.0, 0, 0.0};
    crit = new double[4] {1.0, 1.0, 0.25, 1.0};
    max_int_rel = new double[4] {0.0, 0.0, 0.0, 0.0};
    controller_landing = new ArdronePID(k_p, k_d, k_i, crit, max_int_rel);
    delete[] k_p, k_d, k_i, crit, max_int_rel;

    /*double* k_p = new double[4] {10, 0.5, 0, 5};
    double* k_d = new double[4] {0, 0.3, 0, 1};
    double* k_i = new double[4] {2, 0.0, 0, 0.0};
    double* crit = new double[4] {0.5, 0.5, 0.1, 0.5};
    double* max_int_rel = new double[6] {0.5, 0.0, 0.25, 0.0};
    controller_chasing = new ArdronePID(hz, k_p, k_d, k_i, crit, max_int_rel);
    
    k_p = new double[4] {0.25, 0.25, 0.1, 0.5};
    k_d = new double[4] {0.0, 0.0, 0, 0};
    k_i = new double[4] {0.0, 0.0, 0, 0.0};
    crit = new double[4] {0.15, 0.15, 0.15, 0.15};
    max_int_rel = new double[6] {0.0, 0.0, 0.0, 0.0};
    controller_landing = new ArdronePID(hz, k_p, k_d, k_i, crit, max_int_rel);
    delete[] k_p, k_d, k_i, crit, max_int_rel;*/

    is_hovering = false;

    controller = controller_landing;
}


ArdroneARTag::~ArdroneARTag()
{
    sub_ar_tag.shutdown();
    pub_twist.shutdown();
}


void ArdroneARTag::correct_necessary_pose_shift(void)
{
    //ROS_INFO("necessary shift: x: %f, y: %f, z: %f, yaw: %f, state: %d, bottom: %d", necessary_pose_shift.x, necessary_pose_shift.y, necessary_pose_shift.z, necessary_pose_shift.rot_z, pose_handler->get_state(), is_spotted_bottom);
    current_pose_filter->filter_pose(pose_handler->get_pose_rpy());
    current_pose = current_pose_filter->get_filtered_pose();
    PoseRPY delta_pose = current_pose - current_spotted_pose;
    PoseRPY necessary_shift_global = ArdronePoseHandler::local_to_global_shifted(current_camera_object_distance, current_spotted_pose.rot_z) - delta_pose;
    double dt = (ros::Time::now() - last_spotted_time).toNSec() / 1e+9;
    if (is_spotted_bottom || is_spotted_front)
    {
        necessary_shift_global.x += limit(1.0 * camera_distance_velocity.x * dt, -5, 5);
        necessary_shift_global.y += limit(1.0 * camera_distance_velocity.y * dt, -5, 5);
    }
    necessary_pose_shift_global->filter_pose(necessary_shift_global);
    necessary_pose_shift = ArdronePoseHandler::global_to_local_shifted(necessary_pose_shift_global->get_filtered_pose(), current_pose.rot_z);
    //if (is_spotted_front)
    //    necessary_pose_shift.rot_z = atan(necessary_pose_shift.y / necessary_pose_shift.x);
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
    necessary_pose_shift = PoseRPY::transform_pose_zyx(necessary_pose_shift,
                                                       pose_handler->get_pose_rpy().rot_x,
                                                       pose_handler->get_pose_rpy().rot_y,
                                                       0);
}


/*void ArdroneARTag::get_velocity(void)
{
    current_pose_filter->filter_pose(pose_handler->get_pose_rpy());
    current_pose = current_pose_filter->get_filtered_pose();
    PoseRPY delta_necessary_shift = ArdronePoseHandler::local_to_global_shifted(necessary_pose_shift, current_pose.rot_z) - ArdronePoseHandler::local_to_global_shifted(last_necessary_shift, last_spotted_pose.rot_z),
    delta_pose = current_pose - last_spotted_pose;
    velocity_filter->filter_pose((delta_necessary_shift + delta_pose) / ((ros::Time::now() - last_spotted_time).toNSec() / 1e+9));
    velocity = velocity_filter->get_filtered_pose();
    //ROS_INFO("lps x: %f, y: %f", necessary_pose_shift.x, necessary_pose_shift.y);
    ROS_INFO("vel x: %f, y: %f", delta_necessary_shift.x, delta_necessary_shift.y);
}*/


// Measurement of object's relarive velocity
void ArdroneARTag::get_camera_object_velocity(void)
{
    PoseRPY delta_camera_distance = ArdronePoseHandler::local_to_global_shifted(current_camera_object_distance, current_spotted_pose.rot_z)
            - ArdronePoseHandler::local_to_global_shifted(last_camera_object_distance, last_spotted_pose.rot_z);
    camera_velocity_filter->filter_pose((delta_camera_distance + current_spotted_pose - last_spotted_pose) / ((current_spotted_time - last_spotted_time).toNSec() / 1e+9));
    camera_distance_velocity = camera_velocity_filter->get_filtered_pose();
    //ROS_INFO("vel: x = %f, y = %f", camera_distance_velocity.x, camera_distance_velocity.y);
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
    if (ar_tag_position.get_last_time() != current_spotted_time)
    {
        necessary_pose_shift = ar_tag_position.get_pose();
        stabilize_necessary_pose_shift();

        //ROS_INFO("%f, %f, %f, %f", necessary_pose_shift.x, necessary_pose_shift.y, necessary_pose_shift.z, necessary_pose_shift.rot_z);
        //ROS_INFO("%f, %f, %f, %f", ar_tag_position.get_pose().x, ar_tag_position.get_pose().y, ar_tag_position.get_pose().z, ar_tag_position.get_pose().rot_z);

        last_camera_object_distance = current_camera_object_distance;
        current_camera_object_distance = necessary_pose_shift;

        is_spotted_bottom = ar_tag_position.get_is_spotted_bottom();
        is_spotted_front = ar_tag_position.get_is_spotted_front();

        if (is_spotted_front) {
            necessary_pose_shift.z = 1.5 - current_pose.z;
        }
        
        last_spotted_pose = current_spotted_pose;
        current_spotted_pose = last_pose = pose_handler->get_pose_rpy();

        last_necessary_shift = necessary_pose_shift;

        last_spotted_time = current_spotted_time;
        current_spotted_time = ar_tag_position.get_last_time();

        get_camera_object_velocity();

        //ROS_INFO("vel: x = %f, y = %f, rot_x = %f, rot_y = %f", necessary_pose_shift.x, necessary_pose_shift.y,
        //         pose_handler->get_pose_rpy().rot_x, pose_handler->get_pose_rpy().rot_y);
    }
}


void ArdroneARTag::control(void)
{
    current_pose = pose_handler->get_pose_rpy();
    operate_ar_tag_position();
    //ROS_INFO("dt %f", (ros::Time::now() - last_spotted_time).toNSec() / 1e+9);
    //ROS_INFO("%f, %f, %f, %f", pose_handler->get_pose_rpy().x, pose_handler->get_pose_rpy().y, pose_handler->get_pose_rpy().z, pose_handler->get_pose_rpy().rot_z);
    double dt = (ros::Time::now() - current_spotted_time).toNSec() / 1e+9;
    if (state == 2)
    {
        //system("rostopic pub -1 /ardrone/takeoff std_msgs/Empty");
    }
    else if (dt >= 5.0) {
        ar_tag_search();
        is_spotted_front = is_spotted_bottom = 0;
    }
    else if (dt >= 1) {
        ar_tag_lost();
        is_spotted_front = is_spotted_bottom = 0;
    }
    else if (is_spotted_bottom || is_spotted_front)
    {
        //ROS_INFO("necessary shift: x: %f, y: %f, z: %f, yaw: %f, state: %d, bottom: %d", necessary_pose_shift.x, necessary_pose_shift.y, necessary_pose_shift.z, necessary_pose_shift.rot_z, pose_handler->get_state(), is_spotted_bottom);
        controller = (is_spotted_bottom) ? controller_landing : controller_chasing;
        if (pose_handler->get_state()!= 8 && pose_handler->get_state()!= 2 && pose_handler->get_pose_rpy().z <= 0.2 && is_spotted_bottom)
        {
            ROS_INFO("landing");
            system("rostopic pub -1 /ardrone/land std_msgs/Empty");
            is_spotted_bottom = is_spotted_front = 0;
        }
        else
            correct_necessary_pose_shift();
    }
    ROS_INFO("necessary shift: x: %f, y: %f, z: %f, yaw: %f, state: %d, bottom: %d", necessary_pose_shift.x, necessary_pose_shift.y, necessary_pose_shift.z, necessary_pose_shift.rot_z, pose_handler->get_state(), is_spotted_bottom);
    //ROS_INFO("%f", (ros::Time::now() - last_time).toNSec() / 1e+9);
    twist = controller->pid_twist(necessary_pose_shift, (ros::Time::now() - last_time).toNSec() / 1e+9 > 0 ?
                                      (ros::Time::now() - last_time).toNSec() / 1e+9 : 0.005);
    //ROS_INFO("necessary twist: x: %f, y: %f, z: %f, yaw: %f", twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z);
    //twist.angular.x = twist.angular.y = 0;
    pub_twist.publish(twist);
    last_time = ros::Time::now();
}


