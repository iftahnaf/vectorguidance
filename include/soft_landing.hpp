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
        double um;
        Eigen::Vector3d gravity;
        
        double soft_landing_tgo_lq(const Eigen::Vector3d r, const Eigen::Vector3d v, double min_tgo=0.01);
        Eigen::Vector3d soft_landing_controller_lq(const Eigen::Vector3d r, const Eigen::Vector3d v, double tgo);
};

#endif