#include "../include/bounded_interception.hpp"
#include "../include/common.hpp"
#include <thread>
#include <chrono>

void integrate_state(double* r, double* v, double* controller, double dt, BoundedInterception &bi) {
    double a[3];

    for (int i = 0; i < 3; ++i) {
        a[i] = (controller[i] - bi.gravity[i]);

        v[i] += a[i] * dt;
        r[i] += v[i] * dt + 0.5 * a[i] * dt * dt;
    }
}

int main(){
    double rp[3] = {0.0, 0.0, 5.0};
    double vp[3] = {0.0, 0.0, 0.0};

    double rt[3] = {100.0, 0.0, 100.0};
    double vt[3] =  {0.0, 0.0, 0.0};

    float r_norm;
    float v_norm;

    BoundedInterception bi;

    float dt = 0.01;
    int counter = 0;

    float tmp_miss_distance = 0;

    for (int i = 0; i < 3; ++i) {
        tmp_miss_distance += std::abs(rt[i]) + std::abs(rp[i]);
    }

    while(true){
        for (int i = 0; i < 3; ++i) {
            bi.r[i] = rt[i] - rp[i];
            bi.v[i] = vt[i] - vp[i];
        }

        r_norm = std::sqrt(bi.r[0]*bi.r[0] + bi.r[1]*bi.r[1] + bi.r[2]*bi.r[2]);
        v_norm = std::sqrt(bi.v[0]*bi.v[0] + bi.v[1]*bi.v[1] + bi.v[2]*bi.v[2]);

        // no second-pass interception
        if (r_norm > tmp_miss_distance){
            break;
        }
        else{
            tmp_miss_distance = r_norm;
        }

        double tgo = bi.bounded_interception_tgo(bi.r, bi.v);
        bi.bounded_interception_controller(bi.r, bi.v, bi.u, tgo);

        integrate_state(rp, vp,  bi.u, dt, bi);

        std::cout << "tgo = " << tgo <<  ", controller = (" << bi.u[0] << ", " << bi.u[1] << ", " << bi.u[2] << "), r = (" << rp[0] << ", " << rp[1] << ", " << rp[2] << "), v = (" << vp[0] << ", " << vp[1] << ", " << vp[2] << ")" << std::endl;
        counter++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "Final Miss Distance: " << r_norm << " [m], Final Miss Velocity: " << v_norm << " [m/s], Total Time: " << counter * dt << " [s]"<< std::endl;
    return 0;
}