#include "../include/soft_landing.hpp"
#include "../include/common.hpp"

SoftLanding::SoftLanding(void){
    this->um = 20.0;
    this->gravity << 0.0, 0.0, -9.81;
}

SoftLanding::~SoftLanding(void){
    return;
}

double SoftLanding::soft_landing_tgo_lq(const Eigen::Vector3d r, const Eigen::Vector3d v, double min_tgo){
    double tgo_f1, tgo_f2, um_1, um_2, tgo;
    Eigen::Matrix<double,5,1> f1;
    Eigen::Matrix<double,5,1> f2;
    
    double rr = r.norm();
    double vv = v.norm();

    f1 << pow(this->um, 2) / 4.0, 0.0, -4.0 * pow(vv, 2), -12.0 * v.dot(r.transpose()), -9.0 * pow(rr, 2);
    f2 << pow(this->um, 2) / 4.0, 0.0, -1.0 * pow(vv, 2), -6.0 * v.dot(r.transpose()), -9.0 * pow(rr, 2);

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

Eigen::Vector3d SoftLanding::soft_landing_controller_lq(const Eigen::Vector3d r, const Eigen::Vector3d v, double tgo){
    Eigen::Vector3d u = (1.0 / (pow(tgo, 2))) * (6.0*r + 4.0*tgo*v) + this->gravity;
    return u;
}

int main(){
    Eigen::Vector3d r, v, controller;

    r << 0.0, 0.0, 110.0;
    v << 0.0, 0.0, -10.0;

    SoftLanding sl;

    double tgo = sl.soft_landing_tgo_lq(r, v);
    controller = sl.soft_landing_controller_lq(r, v, tgo);

    std::cout << "tgo = " << tgo << std::endl;
    std::cout << "controller = " << controller << std::endl;

    return 0;
}