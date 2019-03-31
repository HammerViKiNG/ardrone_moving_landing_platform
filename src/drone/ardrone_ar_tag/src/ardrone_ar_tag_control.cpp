#include "ardrone_ar_tag_control.h"


ArdroneARTag::ArdroneARTag(std::string tf_topic, std::string navdata_topic, std::string cmd_topic, std::string ar_tag_front_topic, std::string ar_tag_bottom_topic, double hz)
{
    sub_tf = nh.subscribe(tf_topic, 1, &ArdroneARTag::tf_callback, this);
    sub_navdata = nh.subscribe(navdata_topic, 1, &ArdroneARTag::navdata_callback, this);
    sub_ar_tag_bottom = nh.subscribe(ar_tag_bottom_topic, 1, &ArdroneARTag::ar_tag_bottom_callback, this);
    pub_twist = nh.advertise<geometry_msgs::Twist>(cmd_topic, 1);
    dt = 1 / hz;
    is_spotted_bottom = false;
    controller = new ArdronePID(hz);
}


void ArdroneARTag::tf_callback(const tf::tfMessage& msg)
{
    linear_coords[0] = msg.transforms[0].transform.translation.x;
    linear_coords[1] = msg.transforms[0].transform.translation.y; 
    //linear_coords[2] = msg.transforms[1].transform.translation.z;
    //ROS_INFO_STREAM(linear_coords[2]);
}


void ArdroneARTag::navdata_callback(const ardrone_autonomy::Navdata& msg)
{
    angular_coords[0] = msg.rotX * M_PI / 180.0;
    angular_coords[1] = msg.rotY * M_PI / 180.0;
    angular_coords[2] = msg.rotZ * M_PI / 180.0;
    linear_coords[2] = msg.altd / 1000;
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
        tf::Matrix3x3(quat).getRPY(necessary_pose[3], necessary_pose[4], necessary_pose[5]);
       
        //necessary_pose[3] = 0;
        //necessary_pose[4] = 0;
        necessary_pose[5] = (necessary_pose[5] < -M_PI / 2.0) ? 1.5 * M_PI + necessary_pose[5] : necessary_pose[5] - M_PI / 2.0;
        if (msg.markers[index].id == 8)
        {
	    necessary_pose[0] = msg.markers[index].pose.pose.position.x / 9.0;
            necessary_pose[1] = msg.markers[index].pose.pose.position.y / 9.0;
            necessary_pose[2] = msg.markers[index].pose.pose.position.z / 9.0;
        }
        else
        {
            necessary_pose[0] = msg.markers[index].pose.pose.position.x;
            necessary_pose[1] = msg.markers[index].pose.pose.position.y;
            necessary_pose[2] = msg.markers[index].pose.pose.position.z;
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
        ROS_INFO("%f, %f, %f", necessary_pose[2], necessary_pose[4], necessary_pose[5]);
        //ROS_INFO("r: %f, %f, %f", angular_coords[0], angular_coords[1], angular_coords[2]);
        twist = controller->pid(necessary_pose);
        //ROS_INFO("twist x: %f, y: %f, z: %f", twist.linear.x, twist.linear.y, twist.linear.z);
        pub_twist.publish(twist);
        if (state != 8 && necessary_pose[2] >= -0.2)
            system("rostopic pub -1 /ardrone/land std_msgs/Empty");
    }
    else 
    {
    }
}


