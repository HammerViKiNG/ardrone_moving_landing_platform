#include "camera_stabilized/image_stabilization.h"


ImageStabilizer::ImageStabilizer(std::string sub_topic, std::string pub_topic) : it_(nh_)
{
    image_sub_ = it_.subscribe(sub_topic, 1, &ImageStabilizer::image_callback, this);
    image_pub_ = it_.advertise(pub_topic, 1);
}


void ImageStabilizer::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    image_pub_.publish(cv_ptr->toImageMsg());
}


cv_bridge::CvImagePtr ImageStabilizer::stabilize_image(const cv_bridge::CvImagePtr& img)
{
}
