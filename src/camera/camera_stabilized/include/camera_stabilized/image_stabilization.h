#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


class ImageStabilizer
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    sensor_msgs::ImageConstPtr curr_image;

    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    cv_bridge::CvImagePtr stabilize_image(const cv_bridge::CvImagePtr& img);

    public:
        ImageStabilizer(std::string sub_topic, std::string pub_topic);
        sensor_msgs::ImageConstPtr get_curr_image() const {return curr_image;};
};
