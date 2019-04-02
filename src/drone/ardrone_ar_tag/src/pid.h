#ifndef PID_H
#define PID_H

#define limit(value, min, max) (value < min) ? min : (value > max) ? max : value
#define handle_angle(angle) (angle >= 0) ? angle * M_PI / 180.0 : M_PI * (2 - angle / 180.0) 
#define sign(a) (a > 0) - (a < 0)

class PID
{
    public:
        PID(double hz, double k_p, double k_d, double k_i, double crit, double max_int_rel);

        double pid(double e);
        void reset_data(void);
    
    private:
        double div(double e, double prev_e, double dt);
        void integr(double& int_e, double e, double dt);

        double K_P, K_D, K_I, crit, max_int;

        double dt;
        double e, prev_e, int_e, div_e;      
};

#endif
