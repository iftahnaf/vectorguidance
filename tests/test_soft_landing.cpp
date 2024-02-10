#include "../include/soft_landing.hpp"
#include <gtest/gtest.h>

TEST(SoftLandingTest, TgoCalculation) {
    SoftLanding sl;

    double r[3] = {0.0, 0.0, 110.0};
    double v[3] = {0.0, 0.0, -10.0};

    double actual_tgo = sl.soft_landing_tgo_lq(r, v);

    EXPECT_GT(actual_tgo, 0.0);
}

TEST(SoftLandingTest, ControllerCalculation) {
    SoftLanding sl;

    double r[3] = {0.0, 0.0, 110.0};
    double v[3] = {0.0, 0.0, -10.0};

    double tgo = sl.soft_landing_tgo_lq(r, v);
    sl.soft_landing_controller_lq(r, v, sl.u, tgo);

    EXPECT_LE(std::sqrt(sl.u[0]*sl.u[0] + sl.u[1]*sl.u[1] + sl.u[2]*sl.u[2]), sl.um);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}