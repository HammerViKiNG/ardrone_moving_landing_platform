#include "pid.h"


PID::PID(double hz, double k_p, double k_d, double k_i, double crit, double max_int_rel)
{
    reset_data();
    K_P = k_p;
    K_D = k_d;
    K_I = k_i;
    this->crit = crit;
    this->max_int = (K_I != 0) ? crit / K_I * max_int_rel : -1;
    dt = 1 / hz;
}


void PID::reset_data(void)
{
    int_e = 0;
    div_e = 0;
}


double PID::pid(double e)
{
    div_e = div(e, prev_e, dt);
    integr(int_e, e, dt);
    prev_e = e;
    return limit(K_P * e + K_D * div_e + K_I * int_e, -crit, crit);
}


double PID::div(double e, double prev_e, double dt)
{
    return (e - prev_e) / dt;
}


void PID::integr(double& int_e, double e, double dt)
{
    int_e += e * dt;
    if (max_int != -1)
        int_e = limit(int_e, -max_int, max_int);
}


