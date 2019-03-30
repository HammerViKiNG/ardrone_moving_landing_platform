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
    linear_coords[2] = msg.altd / 1000.0;
    state = msg.state;
}


void ArdroneARTag::ar_tag_bottom_callback(const ar_track_alvar_msgs::AlvarMarkers& msg)
{
    if (msg.markers.size() && msg.markers[0].id == 4)
    {
        double* temp_ar_tag_coords = new double[3] {
            msg.markers[0].pose.pose.position.x,
            msg.markers[0].pose.pose.position.y,
            msg.markers[0].pose.pose.position.z,
        };
        double* local_ar_tag_coords = new double[2]
        {
            linear_coords[2] * tan(atan(temp_ar_tag_coords[0] / temp_ar_tag_coords[2]) - angular_coords[0]),
            linear_coords[2] * tan(-atan(temp_ar_tag_coords[1] / temp_ar_tag_coords[2]) - angular_coords[1]),
        };
        ar_tag_coords[0] = linear_coords[0] + 0.15 * cos(angular_coords[2]) + local_ar_tag_coords[1] * cos(angular_coords[2]) + local_ar_tag_coords[0] * sin(angular_coords[2]);
        ar_tag_coords[1] = linear_coords[1] + 0.15 * sin(angular_coords[2]) - local_ar_tag_coords[1] * sin(angular_coords[2]) + local_ar_tag_coords[0] * cos(angular_coords[2]);
        //ROS_INFO("%f, %f, %f", local_ar_tag_coords[1] - 0.15 * sin(angular_coords[2]), local_ar_tag_coords[0] - 0.15 * cos(angular_coords[2]), linear_coords[2]);
        geometry_msgs::PoseStamped pose;
        listener.transformPose("base_link", ros::Time::now(), msg.markers[0].pose, "bottom_link", pose);
        ROS_INFO("x: %f, y: %f, z: %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        is_spotted_bottom = true;
        delete temp_ar_tag_coords, local_ar_tag_coords;
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
        necessary_coords[0] = ar_tag_coords[0];
        necessary_coords[1] = ar_tag_coords[1];
        necessary_coords[2] = limit(linear_coords[2] - 10 * dt, 0, linear_coords[2]);
        //ROS_INFO("n: %f, %f, %f", necessary_coords[0], necessary_coords[1], necessary_coords[2]);
        //ROS_INFO("r: %f, %f, %f", angular_coords[0], angular_coords[1], angular_coords[2]);
        twist = controller->pid(linear_coords, angular_coords, necessary_coords);
        //pub_twist.publish(twist);
        //if (state != 8)
            //system("rostopic pub -1 /ardrone/land std_msgs/Empty");
    }
    else 
    {
    }
}


