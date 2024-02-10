#include "../include/soft_landing.hpp"
#include "../include/common.hpp"
#include <thread>
#include <chrono>

void integrate_state(double* r, double* v, double* controller, double dt, SoftLanding &sl) {
    double a[3];

    for (int i = 0; i < 3; ++i) {
        a[i] = (controller[i] - sl.gravity[i]);

        v[i] += a[i] * dt;
        r[i] += v[i] * dt + 0.5 * a[i] * dt * dt;
    }
}

int main(){
    double r_norm, v_norm;

    double rp[3] = {1000.0, 0.0, 2000.0};
    double vp[3] = {-30.0, 0.0, -10.0};

    double rt[3] = {0.0, 0.0, 0.0};
    double vt[3] = {0.0, 0.0, 0.0};

    SoftLanding sl;

    float dt = 0.01;
    int counter = 0;

    float tmp_miss_distance = 0;

    for (int i = 0; i < 3; ++i) {
        tmp_miss_distance += std::abs(rt[i]) + std::abs(rp[i]);
    }

    while(true){
        for (int i = 0; i < 3; ++i) {
            sl.r[i] = rt[i] - rp[i];
            sl.v[i] = vt[i] - vp[i];
        }

        r_norm = std::sqrt(sl.r[0]*sl.r[0] + sl.r[1]*sl.r[1] + sl.r[2]*sl.r[2]);
        v_norm = std::sqrt(sl.v[0]*sl.v[0] + sl.v[1]*sl.v[1] + sl.v[2]*sl.v[2]);

        // no second-pass interception
        if (r_norm > tmp_miss_distance){
            break;
        }
        else{
            tmp_miss_distance = r_norm;
        }

        double tgo = sl.soft_landing_tgo_lq(sl.r, sl.v);
        sl.soft_landing_controller_lq(sl.r, sl.v, sl.u, tgo);

        integrate_state(rp, vp,  sl.u, dt, sl);

        std::cout << "tgo = " << tgo <<  ", controller = (" << sl.u[0] << ", " << sl.u[1] << ", " << sl.u[2] << "), r = (" << rp[0] << ", " << rp[1] << ", " << rp[2] << "), v = (" << vp[0] << ", " << vp[1] << ", " << vp[2] << ")" << std::endl;
        counter++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "Final Miss Distance: " << r_norm << " [m], Final Miss Velocity: " << v_norm << " [m/s], Total Time: " << counter * dt << " [s]"<< std::endl;
    return 0;
}