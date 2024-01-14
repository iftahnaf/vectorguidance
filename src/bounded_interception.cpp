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

double BoundedInterception::bounded_interception_tgo(const Eigen::Vector3d r, const Eigen::Vector3d v, double min_tgo){
    double tgo, drho;
    Eigen::Matrix<double,5,1> f;

    drho = this->rho_u - this->rho_w;

    double rr = r.norm();
    double vv = v.norm();

    f << pow(drho, 2) / 4.0, 0.0, -1.0 * pow(vv, 2), -2.0 * v.dot(r.transpose()), -1.0 * pow(rr, 2);
    tgo = find_minimum_positive_real_root(f);

    return std::max(tgo, min_tgo);
}

Eigen::Vector3d BoundedInterception::bounded_interception_controller(const Eigen::Vector3d r, const Eigen::Vector3d v, double tgo){
    Eigen::Vector3d u = (this->rho_u) * (r + tgo*v) / ((r + tgo*v).norm()) + this->gravity;
    return u;
}