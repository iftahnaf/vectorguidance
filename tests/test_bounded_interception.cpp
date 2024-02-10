#include "../include/bounded_interception.hpp"
#include <gtest/gtest.h>

TEST(BoundedInterceptionTest, TgoCalculation) {
    BoundedInterception bi;

    double r[3] = {100.0, 0.0, 110.0};
    double v[3] = {0.0, 0.0, -10.0};

    double actual_tgo = bi.bounded_interception_tgo(r, v);

    EXPECT_GT(actual_tgo, 0.0);
}

TEST(BoundedInterceptionTest, ControllerCalculation) {
    BoundedInterception bi;

    double r[3] = {100.0, 0.0, 110.0};
    double v[3] = {0.0, 0.0, -10.0};

    double tgo = bi.bounded_interception_tgo(r, v);
    bi.bounded_interception_controller(r, v, bi.u, tgo);

    EXPECT_LE(sqrt(bi.u[0]*bi.u[0] + bi.u[1]*bi.u[1]+ bi.u[2]*bi.u[2]), bi.rho_u + bi.gravity.norm());
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}