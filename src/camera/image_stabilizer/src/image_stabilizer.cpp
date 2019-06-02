#include "image_stabilizer/image_stabilizer.h"


ImageStabilizer::ImageStabilizer(std::string sub_topic, std::string pub_topic) : it(nh), 
                                                                                 x(0), y(0), a(0), 
                                                                                 Q(pstd,pstd,pstd), R(cstd,cstd,cstd),
                                                                                 T(2, 3, CV_64F),
                                                                                 X(0, 0, 0), P(1, 1, 1)
{
    image_sub = it.subscribe(sub_topic, 1, &ImageStabilizer::image_callback, this);
    image_pub = it.advertise(pub_topic, 1);
    sensor_msgs::ImageConstPtr first_img_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(sub_topic);
    cv_bridge::CvImagePtr init_ptr = ros_to_cv(first_img_ptr);
    while (init_ptr->image.empty())
    {
        image_pub.publish(init_ptr->toImageMsg());
        first_img_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(sub_topic);
        init_ptr = ros_to_cv(first_img_ptr);
    }
    prev = init_ptr->image;
    cv::cvtColor(prev, prev_gray, cv::COLOR_BGR2GRAY);
    vert_border = HORIZONTAL_BORDER_CROP * prev.rows / prev.cols;
    ROS_INFO("Finished");
}


void ImageStabilizer::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = ros_to_cv(msg);
    if (!cv_ptr->image.empty())
    {
        cv_ptr->image = stabilize_image(cv_ptr->image);
    }
    image_pub.publish(cv_ptr->toImageMsg());
}


cv_bridge::CvImagePtr ImageStabilizer::ros_to_cv(const sensor_msgs::ImageConstPtr& image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("CV bridge exception: %s", e.what());
        return cv_bridge::CvImagePtr();
    }
    return cv_ptr;
}


cv::Mat ImageStabilizer::stabilize_image(const cv::Mat& curr)
{
    try
    {
        cv::cvtColor(curr, curr_gray, cv::COLOR_BGR2GRAY);

    	std::vector <cv::Point2f> prev_corner_temp, curr_corner_temp;
        std::vector <cv::Point2f> prev_corner, curr_corner;
        std::vector <uchar> status;
        std::vector <float> err;

        cv::goodFeaturesToTrack(prev_gray, prev_corner_temp, 200, 0.01, 30);
        cv::calcOpticalFlowPyrLK(prev_gray, curr_gray, prev_corner_temp, curr_corner_temp, status, err);

	    for(size_t i = 0; i < status.size(); i++) 
        {
            if(status[i]) 
            {
                prev_corner.push_back(prev_corner_temp[i]);
                curr_corner.push_back(curr_corner_temp[i]);
            }
        }
    
        T = cv::estimateRigidTransform(prev_corner, curr_corner, false); // false = rigid transform, no scaling/shearing
    
        if(T.data == NULL) 
            last_T.copyTo(T);
        T.copyTo(last_T);

        double dx = T.at<double>(0,2);
        double dy = T.at<double>(1,2);
        double da = atan2(T.at<double>(1,0), T.at<double>(0,0));

        x += dx;
        y += dy;
        a += da;

        z = Trajectory(x,y,a);
        X_ = X; //X_(k) = X(k-1);
        P_ = P+Q; //P_(k) = P(k-1)+Q;

        K = P_/( P_ + R ); //gain;K(k) = P_(k)/( P_(k)+R );
        X = X_+K * (z - X_); //z-X_ is residual,X(k) = X_(k)+K(k)*(z(k)-X_(k)); 
        P = (Trajectory(1,1,1)-K) * P_; //P(k) = (1-K(k))*P_(k);

        double diff_x = X.x - x;
	    double diff_y = X.y - y;
	    double diff_a = X.a - a;

	    dx = dx + diff_x;
	    dy = dy + diff_y;
	    da = da + diff_a;

	    T.at<double>(0,0) = cos(da);
	    T.at<double>(0,1) = -sin(da);
	    T.at<double>(1,0) = sin(da);
	    T.at<double>(1,1) = cos(da);

	    T.at<double>(0,2) = dx;
	    T.at<double>(1,2) = dy;
    
	    cv::Mat curr2;
	    
	    cv::warpAffine(prev, curr2, T, curr.size());
    
        curr.copyTo(prev);
	    curr_gray.copyTo(prev_gray);

	    //curr2 = curr2(cv::Range(vert_border, curr2.rows - vert_border), cv::Range(HORIZONTAL_BORDER_CROP, curr2.cols - HORIZONTAL_BORDER_CROP));
    
	    //cv::resize(curr2, curr2, curr.size());

        return curr2;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Image stabilizing exception: %s", e.what());
        return curr;
    }
}
