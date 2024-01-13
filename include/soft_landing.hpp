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
        Eigen::Vector3d controller;
        Eigen::Vector3d gravity;
        Eigen::Vector3d pursuit_position;
        Eigen::Vector3d evader_position;
        Eigen::Vector3d pursuit_velocity;
        Eigen::Vector3d evader_velocity;
        
        double soft_landing_tgo_lq(const Eigen::Vector3d r, const Eigen::Vector3d v , double um, Eigen::Vector3d g, double min_tgo=0.01);
        Eigen::Vector3d soft_landing_controller_lq(const Eigen::Vector3d r, const Eigen::Vector3d v, double tgo, Eigen::Vector3d g);
};

#endif