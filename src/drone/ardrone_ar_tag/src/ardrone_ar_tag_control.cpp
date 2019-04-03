#include "ardrone_ar_tag_control.h"


ArdronePoseHandler::ArdronePoseHandler(std::string navdata_topic)
{
    sub_navdata = nh.subscribe(navdata_topic, 1, &ArdronePoseHandler::navdata_callback, this);
    boost::shared_ptr<ardrone_autonomy::Navdata const> start_data = ros::topic::waitForMessage<ardrone_autonomy::Navdata>(navdata_topic, nh);
    last_time = start_data->tm / 1000000.0;
    pose_rpy.x = pose_rpy.y = 0;
    pose_rpy.z = start_data->altd / 1000.0;
    pose_rpy.rot_x = start_data->rotX * M_PI / 180.0;
    pose_rpy.rot_y = start_data->rotY * M_PI / 180.0;
    pose_rpy.rot_z = start_data->rotZ * M_PI / 180.0;
}


PoseRPY ArdronePoseHandler::get_pose_rpy(void)
{
    return pose_rpy;
}


int8_t ArdronePoseHandler::get_state(void)
{
    return state;
}


void ArdronePoseHandler::navdata_callback(const ardrone_autonomy::Navdata& msg)
{
    pose_rpy.rot_x = msg.rotX * M_PI / 180.0;
    pose_rpy.rot_y = msg.rotY * M_PI / 180.0;
    pose_rpy.rot_z = msg.rotZ * M_PI / 180.0;
    pose_rpy.z = msg.altd / 1000.0;
    double curr_time = msg.tm / 1000000.0;
    odometry(msg.vx, msg.vy, curr_time - last_time);
    state = msg.state;
    last_time = curr_time;
}


void ArdronePoseHandler::odometry(double vx, double vy, double dt)
{
    pose_rpy.x += ((cos(pose_rpy.rot_z) * vx + sin(pose_rpy.rot_z) * vy) * dt) / 1000.0;
    pose_rpy.y += ((sin(pose_rpy.rot_z) * vx - cos(pose_rpy.rot_z) * vy) * dt) / 1000.0;
}


ArdroneARTag::ArdroneARTag(std::string navdata_topic, std::string cmd_topic, std::string ar_tag_front_topic, std::string ar_tag_bottom_topic, double hz)
{
    sub_ar_tag_bottom = nh.subscribe(ar_tag_bottom_topic, 1, &ArdroneARTag::ar_tag_bottom_callback, this);
    sub_ar_tag_front = nh.subscribe(ar_tag_front_topic, 1, &ArdroneARTag::ar_tag_front_callback, this);
    pub_twist = nh.advertise<geometry_msgs::Twist>(cmd_topic, 1);

    dt = 1 / hz;

    is_spotted_bottom = is_spotted_front = false;

    double* k_p = new double[6] {0.7, 0.7, 1, 0.1, 0.1, 1};
    double* k_d = new double[6] {0.15, 0.15, 0.15, 0.05, 0.05, 0.2};
    double* k_i = new double[6] {0.02, 0.02, 0, 0.01, 0.01, 0.03};
    double* crit = new double[6] {3, 3, 2, 1, 1, 1};
    double* max_int_rel = new double[6] {0.25, 0.25, 0.1, 0.25, 0.25, 0.3};
    controller = new ArdronePID(hz, k_p, k_d, k_i, crit, max_int_rel);

    pose_handler = new ArdronePoseHandler(navdata_topic);
    current_pose = pose_handler->get_pose_rpy();

    delete k_p, k_d, k_i, crit, max_int_rel;
}


void ArdroneARTag::correct_necessary_pose_shift(void)
{
    current_pose = pose_handler->get_pose_rpy();
    double d_rot_z = current_pose.rot_z - last_pose.rot_z, 
           necessary_pose_shift_x = necessary_pose_shift.x,
           necessary_pose_shift_y = necessary_pose_shift.y;

    necessary_pose_shift.x =  necessary_pose_shift_x * cos(d_rot_z) + necessary_pose_shift_y * sin(d_rot_z);
    necessary_pose_shift.y = -necessary_pose_shift_x * sin(d_rot_z) + necessary_pose_shift_y * cos(d_rot_z);
    necessary_pose_shift.rot_x = -current_pose.rot_x;
    necessary_pose_shift.rot_y = -current_pose.rot_y;
    necessary_pose_shift.rot_z -= d_rot_z;
    necessary_pose_shift.z = -current_pose.z;
    if (is_spotted_front)
        necessary_pose_shift.z += 2;
    last_pose = current_pose;
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
        if (!is_spotted_bottom)
            controller->reset_data();
        is_spotted_bottom = true;
        is_spotted_front = false;
        last_pose = current_pose;
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
        necessary_pose_shift.z = msg.markers[index].pose.pose.position.z;
        necessary_pose_shift.rot_z = atan(necessary_pose_shift.y / necessary_pose_shift.x);
        if (!is_spotted_front)
            controller->reset_data();
        is_spotted_front = true;
        is_spotted_bottom = false;
        last_pose = current_pose;
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
        ROS_INFO("%d, %f, %d", pose_handler->get_state(), current_pose.z, is_spotted_bottom);
        correct_necessary_pose_shift();
        if (pose_handler->get_state()!= 8 && pose_handler->get_state()!= 2 && current_pose.z <= 0.2 && is_spotted_bottom)
            system("rostopic pub -1 /ardrone/land std_msgs/Empty");
        twist = controller->pid_twist(necessary_pose_shift);
        pub_twist.publish(twist);
    }
    else 
    {
    }
}


