#ifndef _BOUNDED_INTERCEPTION_HPP
#define _BOUNDED_INTERCEPTION_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/Polynomials>
#include "math.h"
#include <cmath>
#include "stdio.h"
#include <iostream>

class BoundedInterception{
    private:

    public:
        BoundedInterception(void);
        ~BoundedInterception(void);
        double rho_u;
        double rho_w;
        double r[3];
        double v[3];
        double u[3];
        double gravity[3];
        
        double bounded_interception_tgo(double* r, double* v, double min_tgo=0.01);
        void bounded_interception_controller(double* r, double* v, double* u, double tgo);
};

#endif