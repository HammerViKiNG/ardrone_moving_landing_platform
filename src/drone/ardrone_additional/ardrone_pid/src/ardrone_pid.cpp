#include "ardrone_pid/ardrone_pid.h"


ArdronePID::ArdronePID(double* k_p, double* k_d, double* k_i, double* crit, double* max_int_rel)
{
    for (size_t i = 0; i < 4; i++)  
        controller[i] = new PID(k_p[i], k_d[i], k_i[i], crit[i], max_int_rel[i]);

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


geometry_msgs::Twist ArdronePID::pid_twist(PoseRPY e, double dt)
{
    double temp = controller[0]->pid(e.x, dt);
    twist.linear.x = temp;
    temp = controller[0]->pid(e.y, dt);
    twist.linear.y = temp;
    twist.linear.z = controller[2]->pid(e.z, dt);
    twist.angular.z = controller[3]->pid(e.rot_z, dt);
    return twist;
}




