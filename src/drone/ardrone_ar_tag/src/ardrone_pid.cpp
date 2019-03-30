#include "ardrone_pid.h"


ArdronePID::ArdronePID(double hz)
{
    dt = 1 / hz;
}


geometry_msgs::Twist ArdronePID::pid(double* linear_coords, 
                     double* angular_coords, 
                     double* necessary_coords)
{
    double* d = new double[3];
    double* rot = new double[3];
    for (size_t i = 0; i < 3; i++)
    { 
        d[i] = necessary_coords[i] - linear_coords[i];
        rot[i] = handle_angle(angular_coords[i]);
    }
    double* orient = new double[3] { atan(d[1] / d[2]) - rot[1],
                                   atan(d[0] / d[2]) - rot[2],
                                   atan(d[1] / d[0]) - rot[0]};
    double* d_1 = new double[2] {sqrt(pow(d[0], 2) + pow(d[1], 2)) * cos(orient[2]),
                               sqrt(pow(d[0], 2) + pow(d[1], 2)) * sin(orient[2])};
    e[0] = sign(d[0]) * sqrt(pow(d_1[0], 2) + pow(d[2], 2)) * sin(orient[0]);
    e[1] = sign(d[1]) * sqrt(pow(d_1[1], 2) + pow(d[2], 2)) * sin(orient[1]);
    e[2] = sign(d[2]) * sqrt( (pow(d_1[0], 2) + pow(d[2], 2)) * pow(cos(orient[0]), 2) + (pow(d_1[1], 2) + pow(d[2], 2)) * pow(cos(orient[1]), 2) );
    for (size_t i = 0; i < 3; i++)
    {
        div_e[i] = div(e[i], prev_e[i], dt);
        integr(int_e[i], e[i], dt);
    }
    twist.linear.x = limit(K_P * e[0] + K_D * div_e[0] + K_I * int_e[0], -1, 1);
    twist.linear.y = limit(K_P * e[1] + K_D * div_e[1] + K_I * int_e[1], -1, 1);
    twist.linear.z = limit(K_P * e[2] + K_D * div_e[2] + K_I * int_e[2], -1, 1);
    for (size_t i = 0; i < 3; i++)
        prev_e[i] = e[i];
    delete rot, orient, d_1, d;
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


