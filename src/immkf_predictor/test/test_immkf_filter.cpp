#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "immkf_predictor/motion_models.h"
#include "immkf_predictor/immkf_filter.h"
#include <vector>

using namespace immkf_predictor;

/**
 * Test IMMKF filter
 */
class IMMKFFilterTest : public ::testing::Test {
protected:
    IMMKFFilter immkf_;

    void SetUp() override {
        // Initialize with position [0, 0, 0]
        Eigen::Vector3d initial_pos{0.0, 0.0, 0.0};
        immkf_.initialize(initial_pos);
    }
};

TEST_F(IMMKFFilterTest, Initialization) {
    auto state = immkf_.getFusedState();
    EXPECT_EQ(state.size(), 6);
    EXPECT_NEAR(state(0), 0.0, 1e-6);
    EXPECT_NEAR(state(1), 0.0, 1e-6);
    EXPECT_NEAR(state(2), 0.0, 1e-6);
}

TEST_F(IMMKFFilterTest, ConstantVelocityDetection) {
    // Simulate constant velocity: moving in X direction
    Eigen::Vector3d measurements[] = {
        {0.0, 0.0, 0.0},
        {0.1, 0.0, 0.0},
        {0.2, 0.0, 0.0},
        {0.3, 0.0, 0.0},
        {0.4, 0.0, 0.0}};

    for (int i = 0; i < 5; i++) {
        immkf_.predict(0.1);
        immkf_.update(measurements[i]);
    }

    auto probs = immkf_.getModelProbabilities();
    // Constant velocity model should have highest probability
    EXPECT_GT(probs[0], probs[1]);
    EXPECT_GT(probs[0], probs[2]);
}

TEST_F(IMMKFFilterTest, ConstantAccelerationDetection) {
    // Simulate constant acceleration starting from rest
    std::vector<Eigen::Vector3d> measurements;
    double t = 0.0;
    double a = 0.5;  // acceleration m/s^2

    for (int i = 0; i < 10; i++) {
        double x = 0.5 * a * t * t;
        measurements.push_back({x, 0.0, 0.0});
        t += 0.1;
    }

    // First few measurements with nominal CV model
    for (int i = 0; i < 5; i++) {
        immkf_.predict(0.1);
        immkf_.update(measurements[i]);
    }

    // Transition phase - filter starts detecting acceleration
    for (int i = 5; i < 10; i++) {
        immkf_.predict(0.1);
        immkf_.update(measurements[i]);
    }

    auto probs = immkf_.getModelProbabilities();
    // CA or Singer model should have significant probability
    EXPECT_GT(probs[1] + probs[2], probs[0]);
}

TEST_F(IMMKFFilterTest, TrajectoryPrediction) {
    // Set up constant velocity state
    Eigen::Vector3d z{0.1, 0.0, 0.0};
    immkf_.predict(0.1);
    immkf_.update(z);

    z = {0.2, 0.0, 0.0};
    immkf_.predict(0.1);
    immkf_.update(z);

    // Get predicted trajectory
    auto trajectory = immkf_.getFusedState();
    EXPECT_EQ(trajectory.size(), 6);

    // Velocity should be roughly [1, 0, 0]
    EXPECT_GT(trajectory(3), 0.5);  // vx
    EXPECT_LT(trajectory(4), 0.1);  // vy
    EXPECT_LT(trajectory(5), 0.1);  // vz
}

TEST_F(IMMKFFilterTest, ModelProbabilitiesSum) {
    auto probs = immkf_.getModelProbabilities();
    double sum = 0.0;
    for (auto p : probs) {
        EXPECT_GE(p, 0.0);
        EXPECT_LE(p, 1.0);
        sum += p;
    }
    EXPECT_NEAR(sum, 1.0, 1e-6);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
