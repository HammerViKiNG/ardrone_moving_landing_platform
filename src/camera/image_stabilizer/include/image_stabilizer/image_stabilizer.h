#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/video/tracking.hpp"
#include <cv_bridge/cv_bridge.h>


struct TransformParam
{
    TransformParam() {}
    TransformParam(double _dx, double _dy, double _da) :
      dx(_dx), dy(_dy), da(_da) {}

    double dx;
    double dy;
    double da; // angle
};

struct Trajectory
{
    Trajectory() {}
    Trajectory(double _x, double _y, double _a) :
      x(_x), y(_y), a(_a) {}
	// "+"
	friend Trajectory operator+(const Trajectory &c1,const Trajectory  &c2){
        return Trajectory(c1.x+c2.x,c1.y+c2.y,c1.a+c2.a);
	}
	//"-"
	friend Trajectory operator-(const Trajectory &c1,const Trajectory  &c2){
		return Trajectory(c1.x-c2.x,c1.y-c2.y,c1.a-c2.a);
	}
	//"*"
	friend Trajectory operator*(const Trajectory &c1,const Trajectory  &c2){
		return Trajectory(c1.x*c2.x,c1.y*c2.y,c1.a*c2.a);
	}
	//"/"
	friend Trajectory operator/(const Trajectory &c1,const Trajectory  &c2){
		return Trajectory(c1.x/c2.x,c1.y/c2.y,c1.a/c2.a);
	}
	//"="
	Trajectory operator =(const Trajectory &rx){
		x = rx.x;
		y = rx.y;
		a = rx.a;
		return Trajectory(x,y,a);
	}

    double x;
    double y;
    double a; // angle
};


class ImageStabilizer
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

    const int HORIZONTAL_BORDER_CROP = 20;

    cv::Mat curr_gray, prev, prev_gray;
	cv::Mat T, last_T;

    int vert_border;

    std::vector <TransformParam> prev_to_cur_transform; 

	double a;
	double x;
	double y;

	std::vector <Trajectory> trajectory; 

	std::vector <Trajectory> smoothed_trajectory; 
	Trajectory X;
	Trajectory X_;
	Trajectory P;
	Trajectory P_;
	Trajectory K;
	Trajectory z;

	double pstd = 4e-3;
	double cstd = 0.025;
	Trajectory Q;
	Trajectory R;

	std::vector <TransformParam> new_prev_to_cur_transform;

    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    cv_bridge::CvImagePtr ros_to_cv(const sensor_msgs::ImageConstPtr& image);
    cv::Mat stabilize_image(const cv::Mat& curr);


    public:
        ImageStabilizer(std::string sub_topic, std::string pub_topic);
};
