#ifndef _BOUNDED_INTERCEPTION_HPP
#define _BOUNDED_INTERCEPTION_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/Polynomials>
#include "math.h"
#include "stdio.h"
#include <iostream>

class BoundedInterception{
    private:

    public:
        BoundedInterception(void);
        ~BoundedInterception(void);
        double rho_u;
        double rho_w;
        Eigen::Vector3d gravity;
        
        double bounded_interception_tgo(const Eigen::Vector3d r, const Eigen::Vector3d v, double min_tgo=0.01);
        Eigen::Vector3d bounded_interception_controller(const Eigen::Vector3d r, const Eigen::Vector3d v, double tgo);
};

#endif