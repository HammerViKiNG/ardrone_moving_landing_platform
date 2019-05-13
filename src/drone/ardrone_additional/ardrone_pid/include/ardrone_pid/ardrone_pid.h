#ifndef ARDRONE_PID_H
#define ARDRONE_PID_H

#include <cmath>

#include "geometry_msgs/Twist.h"

#include "pid/pid.h"
#include "pose_rpy/pose_rpy.h"


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
        geometry_msgs::Twist twist;
};

#endif
