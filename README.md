# Project Name

A brief description of the project.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Installation
Clone the repository:

        git clone https://github.com/iftahnaf/vectorguidance.git

Build:

        mkdir build
        cd build
        cmake -DBUILD_TESTS=ON -DBUILD_EXAMPLES=ON ..
        make

## Usage

```cpp
#include "../include/soft_landing.hpp"
#include "../include/common.hpp"
#include <thread>
#include <chrono>

void integrate_state(Eigen::Vector3d &r, Eigen::Vector3d &v, Eigen::Vector3d &controller, double dt, SoftLanding &sl){
    Eigen::Vector3d u;
    u << controller[0], controller[1], controller[2];

    u = u - sl.gravity;

    v = v + u*dt;
    r = r + v*dt + 0.5*u*dt*dt;

    return;
}

int main(){
    Eigen::Vector3d rp, vp, controller, rt, vt, r, v;

    rp << 240000.0, 0.0, 13000.0;
    vp << -1800.0, 0.0, -10.0;

    rt << 0.0, 0.0, 0.0;
    vt << 0.0, 0.0, 0.0;

    SoftLanding sl;

    float dt = 0.01;
    int counter = 0;

    while(rp[2] > 0.0){
        r = rt - rp;
        v = vt - vp;

        double tgo = sl.soft_landing_tgo_lq(r, v);
        controller = sl.soft_landing_controller_lq(r, v, tgo);

        integrate_state(rp, vp, controller, dt, sl);

        std::cout << "tgo = " << tgo <<  ", controller = (" << controller[0] << ", " << controller[1] << ", " << controller[2] << "), r = (" << rp[0] << ", " << rp[1] << ", " << rp[2] << "), v = (" << vp[0] << ", " << vp[1] << ", " << vp[2] << ")" << std::endl;
        counter++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "Final Miss Distance: " << r.norm() << " [m], Final Miss Velocity: " << v.norm() << " [m/s], Total Time: " << counter * dt << " [s]"<< std::endl;
    return 0;
}
```

## Contributing

Guidelines on how to contribute to the project and any code of conduct.

## License

Information about the project's license and any additional terms.
