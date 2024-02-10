#include "../include/soft_landing.hpp"
#include "../include/common.hpp"

SoftLanding::SoftLanding(void){
    this->um = 20.0;
    this->gravity[0] = 0.0;
    this->gravity[1] = 0.0;
    this->gravity[2] = -9.81;
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
double SoftLanding::soft_landing_tgo_lq(double* r, double* v, double min_tgo){
    double tgo_f1, tgo_f2, um_1, um_2, tgo, rr ,vv, rv;

    Eigen::Matrix<double,5,1> f1;
    Eigen::Matrix<double,5,1> f2;
    
    rr = std::sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
    vv = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    rv = r[0]*v[0] + r[1]*v[1] + r[2]*v[2];

    f1 << (this->um * this->um) / 4.0, 0.0, -4.0 * vv * vv, -12.0 * rv, -9.0 * rr * rr;
    f2 << (this->um * this->um) / 4.0, 0.0, -1.0 * vv * vv, -6.0 * rv, -9.0 * rr * rr;

    tgo_f1 = find_minimum_positive_real_root(f1);
    tgo_f2 = find_minimum_positive_real_root(f2);
 
    double max_acceleration_f1[3];
    double max_acceleration_f2[3];

    for (int i = 0; i < 3; ++i) {
        max_acceleration_f1[i] = (2.0 / pow(tgo_f1, 2)) * (3.0*r[i] + 2.0*tgo_f1*v[i]) + this->gravity[i];
        max_acceleration_f2[i] = (-2.0 / pow(tgo_f1, 2)) * (3.0*r[i] + 1.0*tgo_f1*v[i]) + this->gravity[i];
    }

    um_1 = std::sqrt(max_acceleration_f1[0]*max_acceleration_f1[0] + max_acceleration_f1[1]*max_acceleration_f1[1] + max_acceleration_f1[2]*max_acceleration_f1[2]);
    um_2 = std::sqrt(max_acceleration_f2[0]*max_acceleration_f2[0] + max_acceleration_f2[1]*max_acceleration_f2[1] + max_acceleration_f2[2]*max_acceleration_f2[2]);

    tgo = (um_1 > um_2) ? tgo_f1 : tgo_f2;

    return std::max(tgo, min_tgo);
}

double SoftLanding::soft_landing_tgo_bounded(double* r, double* v, double min_tgo){
    double tgo, rr, rv, vv;
    Eigen::Matrix<double,5,1> f;

    rr = std::sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
    vv = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

    rv = r[0]*v[0] + r[1]*v[1] + r[2]*v[2];

    f << (this->um * this->um) / 4.0, 0.0, -4.0 * vv * vv, -12.0 * rv, -9.0 * rr * rr;
    tgo = find_minimum_positive_real_root(f);

    return std::max(tgo, min_tgo);
}

void SoftLanding::soft_landing_controller_lq(double* r, double* v, double* u, double tgo){
    for (int i = 0; i < 3; ++i) {
        u[i] = (1 / (tgo * tgo))*(6*r[i] + 4*tgo*v[i]) + this->gravity[i];
    }
    return;
}

void SoftLanding::soft_landing_controller_bounded(double* r, double* v, double* u, double tgo){
    double zem_norm;

    zem_norm = std::sqrt((3*r[0] + 2*tgo*v[0])*(3*r[0] + 2*tgo*v[0]) + (3*r[1] + 2*tgo*v[1])*(3*r[1] + 2*tgo*v[1]) + (3*r[2] + 2*tgo*v[2])*(3*r[2] + 2*tgo*v[2]));

    for (int i = 0; i < 3; ++i) {
        u[i] = (this->um) * ((3*r[i] + 2*tgo*v[i]) / zem_norm) + this->gravity[i];
    }
    
    return;
}