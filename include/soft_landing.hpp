#ifndef _SOFT_LANDING_HPP
#define _SOFT_LANDING_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/Polynomials>
#include "math.h"
#include "stdio.h"
#include <iostream>

class SoftLanding{
    private:

    public:
        SoftLanding(void);
        ~SoftLanding(void);
        double um;
        double r[3];
        double v[3];
        double u[3];
        double gravity[3];
        
        double soft_landing_tgo_lq(double* r, double* v, double min_tgo=0.01);
        double soft_landing_tgo_bounded(double* r, double* v, double min_tgo=0.01);
        void soft_landing_controller_lq(double* r, double* v, double* u, double tgo);
        void soft_landing_controller_bounded(double* r, double* v, double* u,  double tgo);
};

#endif