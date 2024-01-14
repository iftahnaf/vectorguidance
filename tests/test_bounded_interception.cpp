#include "../include/bounded_interception.hpp"
#include <gtest/gtest.h>

TEST(BoundedInterceptionTest, TgoCalculation) {
    BoundedInterception bi;

    Eigen::Vector3d r, v;
    r << 100.0, 0.0, 110.0;
    v << 0.0, 0.0, -10.0;

    double actual_tgo = bi.bounded_interception_tgo(r, v);

    EXPECT_GT(actual_tgo, 0.0);
}

TEST(BoundedInterceptionTest, ControllerCalculation) {
    BoundedInterception bi;

    Eigen::Vector3d r, v;
    r << 100.0, 0.0, 110.0;
    v << 0.0, 0.0, -10.0;

    double tgo = bi.bounded_interception_tgo(r, v);
    Eigen::Vector3d actual_controller = bi.bounded_interception_controller(r, v, tgo);

    EXPECT_LE(actual_controller.norm(), bi.rho_u + bi.gravity.norm());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}