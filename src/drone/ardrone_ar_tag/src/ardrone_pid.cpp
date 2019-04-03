#include "ardrone_pid.h"


ArdronePID::ArdronePID(double hz, double* k_p, double* k_d, double* k_i, double* crit, double* max_int_rel, ArdronePoseHandler* pose_handler)
{
    for (size_t i = 0; i < 6; i++)  
        controller[i] = new PID(hz, k_p[i], k_d[i], k_i[i], crit[i], max_int_rel[i]);
    dt = 1 / hz;
    this->pose_handler = pose_handler;

    K_P[0] = k_p[0];
    K_D[0] = k_d[0];
    K_I[0] = k_i[0];
    this->crit[0] = crit[0];
    this->max_int[0] = (K_I[0] != 0) ? crit[0] / K_I[0] * max_int_rel[0] : -1;

    K_P[0] = k_p[0];
    K_D[0] = k_d[0];
    K_I[0] = k_i[0];
    this->crit[0] = crit[0];
    this->max_int[0] = (K_I[0] != 0) ? crit[0] / K_I[0] * max_int_rel[0] : -1;
}


ArdronePID::~ArdronePID(void)
{
    for (size_t i = 0; i < 6; i++)
        delete controller[i];
}


void ArdronePID::reset_data(void)
{
    for (size_t i = 0; i < 6; i++)
        controller[i]->reset_data();
}


geometry_msgs::Twist ArdronePID::pid_global(PoseRPY e)
{
    double rot_z = pose_handler->get_pose_rpy().rot_z;

    div_e[0] = div(e.x, prev_e[0], dt);
    integr(int_e[0], e.x, max_int[0], dt);
    prev_e[0] = e.x;

    div_e[1] = div(e.y, prev_e[1], dt);
    integr(int_e[1], e.y, max_int[1], dt);
    prev_e[1] = e.y;

    twist.linear.x = limit(K_P * (e.x * cos(rot_z) + e.y * sin(rot_z))
                                 + K_D * (div_e[0] * cos(rot_z) + div_e[1] * sin(rot_z)
                                 + K_I * (int_e[0] * cos(rot_z) + int_e[1] * sin(rot_z),
                                 -crit[0], crit);
    twist.linear.y = limit(K_P * (e.y * cos(rot_z) - e.x * sin(rot_z))
                                 + K_D * (div_e[1] * cos(rot_z) - div_e[0] * sin(rot_z)
                                 + K_I * (int_e[1] * cos(rot_z) - int_e[0] * sin(rot_z),
                                 -crit, crit);
    twist.linear.z = controller[2]->pid(e.z);
    twist.angular.x = controller[3]->pid(e.rot_x);
    twist.angular.y = controller[4]->pid(e.rot_y);
    twist.angular.z = controller[5]->pid(e.rot_z);
    return twist;
}


double ArdronePID::div(double e, double prev_e, double dt)
{
    return (e - prev_e) / dt;
}


void ArdronePID::integr(double& int_e, double e, max_int, double dt)
{
    int_e += e * dt;
    if (max_int != -1)
        int_e = limit(int_e, -max_int, max_int);
}


double ArdronePID::limit(double value, double min, double max)
{
    return (value < min) ? min : (value > max) ? max : value; 
}



