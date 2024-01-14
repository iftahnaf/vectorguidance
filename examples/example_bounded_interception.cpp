#include "../include/bounded_interception.hpp"
#include "../include/common.hpp"
#include <thread>
#include <chrono>

void integrate_state(Eigen::Vector3d &r, Eigen::Vector3d &v, Eigen::Vector3d &controller, double dt, BoundedInterception &bi){
    Eigen::Vector3d u;
    u << controller[0], controller[1], controller[2];

    u = u - bi.gravity;

    v = v + u*dt;
    r = r + v*dt + 0.5*u*dt*dt;

    return;
}

int main(){
    Eigen::Vector3d rp, vp, controller, rt, vt, r, v;

    rp << 0.0, 0.0, 5.0;
    vp << 0.0, 0.0, 0.0;

    rt << 100.0, 0.0, 100.0;
    vt << 0.0, 0.0, 0.0;

    BoundedInterception bi;

    float dt = 0.01;
    int counter = 0;

    float tmp_miss_distance = rt.norm() + rp.norm();

    while(true){
        r = rt - rp;
        v = vt - vp;

        // no second-pass interception
        if (r.norm() > tmp_miss_distance){
            break;
        }
        else{
            tmp_miss_distance = r.norm();
        }

        double tgo = bi.bounded_interception_tgo(r, v);
        controller = bi.bounded_interception_controller(r, v, tgo);

        integrate_state(rp, vp, controller, dt, bi);

        std::cout << "tgo = " << tgo <<  ", controller = (" << controller[0] << ", " << controller[1] << ", " << controller[2] << "), r = (" << rp[0] << ", " << rp[1] << ", " << rp[2] << "), v = (" << vp[0] << ", " << vp[1] << ", " << vp[2] << ")" << std::endl;
        counter++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "Final Miss Distance: " << r.norm() << " [m], Final Miss Velocity: " << v.norm() << " [m/s], Total Time: " << counter * dt << " [s]"<< std::endl;
    return 0;
}