#include "../include/bounded_interception.hpp"
#include "../include/common.hpp"

BoundedInterception::BoundedInterception(void){
    this->rho_u = 20.0;
    this ->rho_w = 10.0;
    this->gravity << 0.0, 0.0, -9.81;
}

BoundedInterception::~BoundedInterception(void){
    return;
}

/**
 * Calculates the bounded interception time-to-go (TGO) for a given position and velocity vector.
 * 
 * @param r The position vector of the target.
 * @param v The velocity vector of the target.
 * @param min_tgo The minimum time-to-go value.
 * @return The bounded interception time-to-go.
 */
double BoundedInterception::bounded_interception_tgo(double* r, double* v, double min_tgo){
    double tgo, drho;
    Eigen::Matrix<double,5,1> f;
    drho = this->rho_u - this->rho_w;

    double rr = std::sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2]);
    double vv = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

    double rv = r[0]*v[0] + r[1]*v[1] + r[2]*v[2];

    f << pow(drho, 2) / 4.0, 0.0, -1.0 * vv * vv, -2.0 * rv, -1.0 * rr * rr;
    tgo = find_minimum_positive_real_root(f);

    return std::max(tgo, min_tgo);
}

void BoundedInterception::bounded_interception_controller(double* r, double* v, double* u, double tgo) {

    double zem_norm = std::sqrt((r[0] + tgo*v[0])*(r[0] + tgo*v[0]) + (r[1] + tgo*v[1])*(r[1] + tgo*v[1]) + (r[2] + tgo*v[2])*(r[2] + tgo*v[2]));

    for (int i = 0; i < 3; ++i) {
        double zem = r[i] + tgo * v[i];
        double u_direction = zem / zem_norm;
        u[i] = (this->rho_u) * u_direction + this->gravity[i];
    }
    return;
}
