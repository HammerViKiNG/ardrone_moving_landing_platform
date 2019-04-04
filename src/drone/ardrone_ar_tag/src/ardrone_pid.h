#ifndef ARDRONE_PID_H
#define ARDRONE_PID_H

#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

#include "pid.h"
#include "pose_rpy.h"
#include "ardrone_pose_handler.h"


class ArdronePID
{
    public:
        ArdronePID(double hz, double* k_p, double* k_d, double* k_i, double* crit, double* max_int_rel);
        ~ArdronePID(void);

        geometry_msgs::Twist pid_twist(PoseRPY e);
        void reset_data(void);
    
    private:  
        double dt;

        PID* controller[4];

        PoseRPY global_pose;
        ArdronePoseHandler* pose_handler;
        geometry_msgs::Twist twist;
};

#endif
