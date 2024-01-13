#include "../include/soft_landing.hpp"
#include <gtest/gtest.h>

TEST(SoftLandingTest, TgoCalculation) {
    SoftLanding sl;

    Eigen::Vector3d r, v;
    r << 0.0, 0.0, 110.0;
    v << 0.0, 0.0, -10.0;

    double actual_tgo = sl.soft_landing_tgo_lq(r, v);

    EXPECT_LT(actual_tgo, 0);
}

TEST(SoftLandingTest, ControllerCalculation) {
    SoftLanding sl;

    Eigen::Vector3d r, v;
    r << 0.0, 0.0, 110.0;
    v << 0.0, 0.0, -10.0;

    double tgo = sl.soft_landing_tgo_lq(r, v);
    Eigen::Vector3d actual_controller = sl.soft_landing_controller_lq(r, v, tgo);

    EXPECT_ANY_THROW(actual_controller.norm());
}

#ifdef USE_TEST
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#endif
