#include "ardrone_pid.h"


ArdronePID::ArdronePID(double hz, double* k_p, double* k_d, double* k_i, double* crit)
{
    for (size_t i = 0; i < 6; i++)
    {
        prev_e[i] = 0;
        int_e[i] = 0;
        K_P[i] = k_p[i];
        K_D[i] = k_d[i];
        K_I[i] = k_i[i];
        CRIT[i] = crit[i];
    }
    dt = 1 / hz;
}


geometry_msgs::Twist ArdronePID::pid(double* e)
{
    for (size_t i = 0; i < 6; i++)
    {
        div_e[i] = div(e[i], prev_e[i], dt);
        integr(int_e[i], e[i], dt);
    }
    twist.linear.x = limit(K_P[0] * e[0] + K_D[0] * div_e[0] + K_I[0] * int_e[0], -CRIT[0], CRIT[0]);
    twist.linear.y = limit(K_P[1] * e[1] + K_D[1] * div_e[1] + K_I[1] * int_e[1], -CRIT[1], CRIT[1]);
    twist.linear.z = limit(K_P[2] * e[2] + K_D[2] * div_e[2] + K_I[2] * int_e[2], -CRIT[2], CRIT[2]);
    twist.angular.x = limit(K_P[3] * e[3] + K_D[3] * div_e[3] + K_I[3] * int_e[3], -CRIT[3], CRIT[3]);
    twist.angular.y = limit(K_P[4] * e[4] + K_D[4] * div_e[4] + K_I[4] * int_e[4], -CRIT[4], CRIT[4]);
    twist.angular.z = limit(K_P[5] * e[5] + K_D[5] * div_e[5] + K_I[5] * int_e[5], -CRIT[5], CRIT[5]);
    for (size_t i = 0; i < 6; i++)
        prev_e[i] = e[i];
    return twist;
}


double ArdronePID::div(double e, double prev_e, double dt)
{
    return (e - prev_e) / dt;
}


void ArdronePID::integr(double& int_e, double e, double dt)
{
    int_e += e * dt;
}


