#include "ardrone_pid.h"


ArdronePID::ArdronePID(double hz, double* k_p, double* k_d, double* k_i, double* crit, double* max_int_rel)
{
    for (size_t i = 0; i < 4; i++)  
        controller[i] = new PID(hz, k_p[i], k_d[i], k_i[i], crit[i], max_int_rel[i]);
    dt = 1 / hz;
}


ArdronePID::~ArdronePID(void)
{
    for (size_t i = 0; i < 4; i++)
        delete controller[i];
}


void ArdronePID::reset_data(void)
{
    for (size_t i = 0; i < 4; i++)
        controller[i]->reset_data();
}


geometry_msgs::Twist ArdronePID::pid_twist(PoseRPY e)
{

    twist.linear.x = controller[0]->pid(e.x);
    twist.linear.y = controller[1]->pid(e.y);
    twist.linear.z = controller[2]->pid(e.z);
    twist.angular.z = controller[3]->pid(e.rot_z);
    return twist;
}




