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
        ArdronePID(double hz, double* k_p, double* k_d, double* k_i, double* crit);

        geometry_msgs::Twist pid(double* e);
    
    private:
        double div(double e, double prev_e, double dt);
        void integr(double& int_e, double e, double dt);

        double K_P[6], K_D[6], K_I[6], CRIT[6];

        double dt;
        double e[6], prev_e[6], int_e[6], div_e[6];      

        geometry_msgs::Twist twist;
};

#endif
