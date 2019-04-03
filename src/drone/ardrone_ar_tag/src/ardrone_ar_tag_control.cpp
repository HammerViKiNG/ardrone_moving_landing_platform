#include "ardrone_ar_tag_control.h"


ArdroneARTag::ArdroneARTag(std::string navdata_topic, std::string cmd_topic, std::string ar_tag_front_topic, std::string ar_tag_bottom_topic, double hz)
{
    sub_navdata = nh.subscribe(navdata_topic, 1, &ArdroneARTag::navdata_callback, this);
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

    current_pose.x = current_pose.y = 0;

    delete k_p, k_d, k_i, crit, max_int_rel;
}


void ArdroneARTag::navdata_callback(const ardrone_autonomy::Navdata& msg)
{
    double d_rot_z = msg.rotZ * M_PI / 180.0 - current_pose.rot_z, 
           necessary_x = necessary_pose.x,
           necessary_y = necessary_pose.y;

    current_pose.rot_x = msg.rotX * M_PI / 180.0;
    current_pose.rot_y = msg.rotY * M_PI / 180.0;
    current_pose.rot_z = msg.rotZ * M_PI / 180.0;
    current_pose.z = msg.altd / 1000.0;
    state = msg.state;

    necessary_pose.x =  necessary_x * cos(d_rot_z) + necessary_y * sin(d_rot_z);
    necessary_pose.y = -necessary_x * sin(d_rot_z) + necessary_y * cos(d_rot_z);
    necessary_pose.rot_x = -current_pose.rot_x;
    necessary_pose.rot_y = -current_pose.rot_y;
    necessary_pose.rot_z -= d_rot_z;
    necessary_pose.z = -current_pose.z;
    if (is_spotted_front)
        necessary_pose.z += 2;
}


void ArdroneARTag::ar_tag_bottom_callback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
    size_t index = 0;
    if (!msg.markers.empty())
    {
        if (msg.markers.size() == 2 && msg.markers[1].id == 4)
            index = 1;
	    tf::quaternionMsgToTF(msg.markers[index].pose.pose.orientation, quat);
    	tf::Matrix3x3(quat).getRPY(necessary_pose.rot_x, necessary_pose.rot_y, necessary_pose.rot_z);
        necessary_pose.rot_z = (necessary_pose.rot_z < -M_PI / 2.0) ? 1.5 * M_PI + necessary_pose.rot_z : necessary_pose.rot_z - M_PI / 2.0;
        if (msg.markers[index].id == 8)
        {
	        necessary_pose.x = msg.markers[index].pose.pose.position.x / 9.0;
            necessary_pose.y = msg.markers[index].pose.pose.position.y / 9.0;
            necessary_pose.z = msg.markers[index].pose.pose.position.z / 9.0;
        }
        else
        {
            necessary_pose.x = msg.markers[index].pose.pose.position.x;
            necessary_pose.y = msg.markers[index].pose.pose.position.y;
            necessary_pose.z = msg.markers[index].pose.pose.position.z;
        }
        if (!is_spotted_bottom)
            controller->reset_data();
        is_spotted_bottom = true;
        is_spotted_front = false;
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
        tf::Matrix3x3(quat).getRPY(necessary_pose.rot_x, necessary_pose.rot_y, necessary_pose.rot_z);
        necessary_pose.rot_z = atan(msg.markers[index].pose.pose.position.y / msg.markers[index].pose.pose.position.x);
        necessary_pose.x = msg.markers[index].pose.pose.position.x;
        necessary_pose.y = msg.markers[index].pose.pose.position.y;
        necessary_pose.z = msg.markers[index].pose.pose.position.z;
        ROS_INFO("lel");
        if (!is_spotted_front)
            controller->reset_data();
        is_spotted_front = true;
        is_spotted_bottom = false;
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
        twist = controller->pid_twist(necessary_pose);
        ROS_INFO("%f, %f", necessary_pose.z, current_pose.z);
        pub_twist.publish(twist);
        if (state != 8 && current_pose.z <= 0.2 && is_spotted_bottom)
            system("rostopic pub -1 /ardrone/land std_msgs/Empty");
    }
    else 
    {
    }
}


