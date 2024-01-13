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

void explicit_solve(double x, int i);
void explicit_find_minimum_positive_real_root(void);
void find_minimum_positive_real_root(void);

#endif