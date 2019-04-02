#include "ardrone_ar_tag_control.h"


ArdroneARTag::ArdroneARTag(std::string navdata_topic, std::string cmd_topic, std::string ar_tag_front_topic, std::string ar_tag_bottom_topic, double hz)
{
    sub_navdata = nh.subscribe(navdata_topic, 1, &ArdroneARTag::navdata_callback, this);
    sub_ar_tag_bottom = nh.subscribe(ar_tag_bottom_topic, 1, &ArdroneARTag::ar_tag_bottom_callback, this);
    pub_twist = nh.advertise<geometry_msgs::Twist>(cmd_topic, 1);
    dt = 1 / hz;
    is_spotted_bottom = false;
    double* k_p = new double[6] {0.7, 0.7, 0.4, 0.1, 0.1, 1};
    double* k_d = new double[6] {0.15, 0.15, 0.1, 0.05, 0.05, 0.2};
    double* k_i = new double[6] {0.02, 0.02, 0, 0.01, 0.01, 0.03};
    double* crit = new double[6] {3, 3, 1, 1, 1, 1};
    double* max_int_rel = new double[6] {0.25, 0.25, 0.1, 0.25, 0.25, 0.5};
    controller = new ArdronePID(hz, k_p, k_d, k_i, crit, max_int_rel);
    delete k_p, k_d, k_i, crit, max_int_rel;
}


void ArdroneARTag::navdata_callback(const ardrone_autonomy::Navdata& msg)
{
    angular_coords[0] = msg.rotX * M_PI / 180.0;
    angular_coords[1] = msg.rotY * M_PI / 180.0;
    angular_coords[2] = msg.rotZ * M_PI / 180.0;
    z = msg.altd / 1000.0;
    state = msg.state;
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
        is_spotted_bottom = true;
    }
}


void ArdroneARTag::control(void)
{
    if (state == 2)
    {
        //system("rostopic pub -1 /ardrone/takeoff std_msgs/Empty");
    }
    else if (is_spotted_bottom)
    {
        twist = controller->pid_twist(necessary_pose);
        pub_twist.publish(twist);
        if (state != 8 && z <= 0.2)
            system("rostopic pub -1 /ardrone/land std_msgs/Empty");
    }
    else 
    {
    }
}


