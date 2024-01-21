![OS](https://img.shields.io/badge/OS-Linux-red?style=flat&logo=linux)
[![Docker](https://img.shields.io/badge/Docker-available-green.svg?style=flat&logo=docker)](https://github.com/emalderson/ThePhish/tree/master/docker)
[![Maintenance](https://img.shields.io/badge/Maintained-yes-green.svg)](https://github.com/iftahnaf/vectorguidance)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# vectorguidance

Vector Guidance are 3D optimal control methods for aerial systems.

The guidance laws based on a controller that minimized an finite LQ cost function with the form of:

$$ J = \|\mathbf{y(t_f)}\| + k \int_{t_0}^{t_f} \|\mathbf{u(t)}\|^2 dt $$

Where:
- $y$ is the Zero-Effort-Miss (ZEM) / Zero-Effort-Velocity (ZEV) variable.
- $k$ is the weight on the integration part of the cost.
- $u$ is the controller.
- $t_0$ is the initial time and $t_f$ is the final time.

Because the controller that minimized the LQ cost function is unbound, we define the maximum acceleration of the system as $u_m$, such that:

$\|\mathbf{u}\| \leq u_m$ while $t_0 \leq t \leq t_f$

**Note**: The value of $u_m$ is determined by the physical properties of the system (eg. thrusters saturations, aerodynamical constants)

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
