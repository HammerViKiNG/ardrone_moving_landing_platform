#ifndef ARDRONE_PID_H
#define ARDRONE_PID_H

#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include "pid.h"
#include "pose_rpy.h"
#include "ardrone_pose_handler.h"

//#define limit(value, min, max) (value < min) ? min : (value > max) ? max : value


class ArdronePID
{
    public:
        ArdronePID(double hz, double* k_p, double* k_d, double* k_i, double* crit, double* max_int_rel, ArdronePoseHandler* pose_handler);
        ~ArdronePID(void);

        geometry_msgs::Twist pid_global(PoseRPY e);
        void reset_data(void);
    
    private:
        double div(double e, double prev_e, double dt);
        void integr(double& int_e, double e, double max_int, double dt); 
        double limit(double value, double min, double max);
    
        double dt;

        double K_P[2], K_D[2], K_I[2], crit[2], max_int[2];

        double e[2], prev_e[2], int_e[2], div_e[2]; 

        PID* controller[6];

        PoseRPY global_pose;
        ArdronePoseHandler* pose_handler;
        geometry_msgs::Twist twist;
};

#endif
