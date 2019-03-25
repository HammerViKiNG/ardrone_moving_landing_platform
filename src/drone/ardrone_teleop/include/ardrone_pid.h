#ifndef ARDRONE_PID_H
#define ARDRONE_PID_H

#include <cmath>

#include "ros/ros.h"
#include "tf/tfMessage.h"
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"

#define limit(value, min, max) (value < min) ? min : (value > max) ? max : value
#define handle_angle(angle) (angle >= 0) ? angle * M_PI / 180.0 : M_PI * (2 - angle / 180.0) 
#define sign(a) (a > 0) - (a < 0)

class ArdronePID
{
    public:
        ArdronePID(std::string tf_topic, std::string navdata_topic, std::string cmd_topic);
        void set_necessary_coords(double* necessary_coords);
        void control(void);
    
    private:
        void tf_callback(const tf::tfMessage& msg);
        void navdata_callback(const ardrone_autonomy::Navdata& msg);
        void pid(void);
        double div(double e, double prev_e, double dt);
        void integr(double& int_e, double e, double dt);

        const double K_P = 0.5, K_D = 0.1, K_I = 0.1;

        int8_t state;
        double necessary_coords[3], linear_coords[3], angular_coords[3];
        double prev_time, curr_time, dt = 0.00001;
        double e[3], prev_e[3], int_e[3], div_e[3];      
        
        ros::NodeHandle nh;
        ros::Subscriber sub_tf, sub_navdata;
        ros::Publisher pub_tf;

        geometry_msgs::Twist twist;
};

#endif
