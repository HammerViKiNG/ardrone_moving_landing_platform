#ifndef ARDRONE_PID_H
#define ARDRONE_PID_H

#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#define limit(value, min, max) (value < min) ? min : (value > max) ? max : value
#define handle_angle(angle) (angle >= 0) ? angle * M_PI / 180.0 : M_PI * (2 - angle / 180.0) 
#define sign(a) (a > 0) - (a < 0)

class ArdronePID
{
    public:
        ArdronePID(double hz);
        geometry_msgs::Twist pid(double* linear_coords, 
                                 double* angular_coords, 
                                 double* necessary_coords);
        geometry_msgs::Twist pid(double* e);
    
    private:
        double div(double e, double prev_e, double dt);
        void integr(double& int_e, double e, double dt);

        const double K_P = 0.5f, K_D = 0.1f, K_I = 0.05f;

        double dt;
        double e[3], prev_e[3], int_e[3], div_e[3];      

        geometry_msgs::Twist twist;
};

#endif
