#ifndef _COMMON_HPP
#define _COMMON_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/Polynomials>
#include "math.h"
#include "stdio.h"
#include <iostream>

extern double a,b,c,d,e,j,k,l,p,q,t,z;
extern double tgo;

double explicit_solve(double x, int i);
double explicit_find_minimum_positive_real_root(const Eigen::Matrix<double,5,1> coeff);
double find_minimum_positive_real_root(const Eigen::Matrix<double,5,1> coeff);
double find_maximum_positive_real_root(const Eigen::Matrix<double,5,1> coeff);
double tgo_guard(double tgo, double current_tgo);

#endif