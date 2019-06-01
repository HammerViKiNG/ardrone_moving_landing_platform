#include "camera_stabilized/image_stabilization.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageStabilizer image_stabilizer("/ardrone/image_raw", "/ardrone/image_raw_stabilized");
    ros::spin();
    return 0;
}
