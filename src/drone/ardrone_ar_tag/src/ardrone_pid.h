#ifndef ARDRONE_PID_H
#define ARDRONE_PID_H

#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include "pid.h"


struct PoseRPY
{
    double x;
    double y;
    double z;
    double rot_x;
    double rot_y;
    double rot_z;
};


class ArdronePID
{
    public:
        ArdronePID(double hz, double* k_p, double* k_d, double* k_i, double* crit, double* max_int_rel);
        ~ArdronePID(void);

        geometry_msgs::Twist pid_twist(PoseRPY e);
        void reset_data(void);
    
    private:
        double div(double e, double prev_e, double dt);
        void integr(double& int_e, double e, double dt); 

        PID* controller[6];       
        double dt;

        geometry_msgs::Twist twist;
};

#endif
