#include "../include/soft_landing.hpp"
#include "../include/common.hpp"

SoftLanding::SoftLanding(){
    this->um = 20.0;
    this->gravity << 0.0, 0.0, -9.81;
}

SoftLanding::~SoftLanding(){
    return;
}

double SoftLanding::soft_landing_tgo_lq(const Eigen::Vector3d r, const Eigen::Vector3d v , double um, Eigen::Vector3d g, double min_tgo=0.01){
    double tgo_f1, tgo_f2, um_1, um_2, tgo;
    
    double rr = r.norm();
    double vv = v.norm();

    Eigen::Matrix<double,5,1> f1;
    Eigen::Matrix<double,5,1> f2;

    f1 << pow(um, 2) / 4.0, 0.0, -4.0 * pow(vv, 2), -12.0 * v.dot(r.transpose()), -9.0 * pow(rr, 2);
    f2 << pow(um, 2) / 4.0, 0.0, -1.0 * pow(vv, 2), -6.0 * v.dot(r.transpose()), -9.0 * pow(rr, 2);

    try{
        tgo_f1 = explicit_find_minimum_positive_real_root(f1);
        tgo_f2 = explicit_find_minimum_positive_real_root(f2);
    }
    catch(const std::exception& e){
        tgo_f1 = find_minimum_positive_real_root(f1);
        tgo_f2 = find_minimum_positive_real_root(f2);
    }
 
    Eigen::Vector3d max_acceleration_f1 = (2.0 / pow(tgo_f1, 2)) * (3.0*r + 2.0*tgo_f1*v) + this->gravity;
    Eigen::Vector3d max_acceleration_f2 = (-2.0 / pow(tgo_f1, 2)) * (3.0*r + 1.0*tgo_f1*v) + this->gravity;

    um_1 = max_acceleration_f1.norm();
    um_2 = max_acceleration_f2.norm();

    if (um_1 > um_2){
        tgo = tgo_f1;
    }
    else{
        tgo = tgo_f2;
    }

    if (tgo < min_tgo){
        tgo = min_tgo;
    }
    return tgo;
}