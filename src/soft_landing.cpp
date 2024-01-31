#include "../include/soft_landing.hpp"
#include "../include/common.hpp"

SoftLanding::SoftLanding(void){
    this->um = 20.0;
    this->gravity << 0.0, 0.0, -9.81;
}

SoftLanding::~SoftLanding(void){
    return;
}

/**
 * Calculates the time-to-go (TGO) for soft landing using the LQ method.
 * 
 * @param r The position vector.
 * @param v The velocity vector.
 * @param min_tgo The minimum time-to-go value.
 * @return The calculated time-to-go for soft landing.
 */
double SoftLanding::soft_landing_tgo_lq(const Eigen::Vector3d r, const Eigen::Vector3d v, double min_tgo){
    double tgo_f1, tgo_f2, um_1, um_2, tgo;
    Eigen::Matrix<double,5,1> f1;
    Eigen::Matrix<double,5,1> f2;
    
    double rr = r.norm();
    double vv = v.norm();

    f1 << pow(this->um, 2) / 4.0, 0.0, -4.0 * pow(vv, 2), -12.0 * v.dot(r.transpose()), -9.0 * pow(rr, 2);
    f2 << pow(this->um, 2) / 4.0, 0.0, -1.0 * pow(vv, 2), -6.0 * v.dot(r.transpose()), -9.0 * pow(rr, 2);

    tgo_f1 = find_minimum_positive_real_root(f1);
    tgo_f2 = find_minimum_positive_real_root(f2);
 
    Eigen::Vector3d max_acceleration_f1 = (2.0 / pow(tgo_f1, 2)) * (3.0*r + 2.0*tgo_f1*v) + this->gravity;
    Eigen::Vector3d max_acceleration_f2 = (-2.0 / pow(tgo_f1, 2)) * (3.0*r + 1.0*tgo_f1*v) + this->gravity;

    um_1 = max_acceleration_f1.norm();
    um_2 = max_acceleration_f2.norm();

    tgo = (um_1 > um_2) ? tgo_f1 : tgo_f2;

    return std::max(tgo, min_tgo);
}

double SoftLanding::soft_landing_tgo_bounded(const Eigen::Vector3d r, const Eigen::Vector3d v, double min_tgo){
    double tgo_f1, tgo_f2, um_1, um_2, tgo;
    Eigen::Matrix<double,5,1> f1;
    Eigen::Matrix<double,5,1> f2;
    
    double rr = r.norm();
    double vv = v.norm();

    f1 << pow(this->um, 2) / 4.0, 0.0, -4.0 * pow(vv, 2), -12.0 * v.dot(r.transpose()), -9.0 * pow(rr, 2);
    f2 << pow(this->um, 2) / 4.0, 0.0, -1.0 * pow(vv, 2), -6.0 * v.dot(r.transpose()), -9.0 * pow(rr, 2);

    tgo_f1 = find_maximum_positive_real_root(f1);
    tgo_f2 = find_maximum_positive_real_root(f2);
 
    Eigen::Vector3d max_acceleration_f1 = (2.0 / pow(tgo_f1, 2)) * (3.0*r + 2.0*tgo_f1*v) + this->gravity;
    Eigen::Vector3d max_acceleration_f2 = (-2.0 / pow(tgo_f1, 2)) * (3.0*r + 1.0*tgo_f1*v) + this->gravity;

    um_1 = max_acceleration_f1.norm();
    um_2 = max_acceleration_f2.norm();

    // print bool value for um_1 > um_2
    printf("um_1 bigger than um_2: %d", um_1 > um_2);

    tgo = (um_1 > um_2) ? tgo_f1 : tgo_f2;

    return std::max(tgo, min_tgo);
}

Eigen::Vector3d SoftLanding::soft_landing_controller_lq(const Eigen::Vector3d r, const Eigen::Vector3d v, double tgo){
    Eigen::Vector3d u = (1.0 / (pow(tgo, 2))) * (6.0*r + 4.0*tgo*v) + this->gravity;
    return u;
}

Eigen::Vector3d SoftLanding::soft_landing_controller_bounded(const Eigen::Vector3d r, const Eigen::Vector3d v, double tgo){
    Eigen::Vector3d u = (this->um) * (r + tgo*v) / ((r + tgo*v).norm()) + this->gravity;
    return u;
}