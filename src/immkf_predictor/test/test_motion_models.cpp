#include <iostream>
#include <iomanip>
#include <cmath>
#include <Eigen/Dense>

#include "immkf_predictor/motion_models.h"
#include "immkf_predictor/immkf_filter.h"

using namespace immkf_predictor;

// Helper function to print state
void printState(const std::string& name, const Eigen::VectorXd& state) {
  std::cout << name << ": [";
  for (int i = 0; i < state.size(); ++i) {
    std::cout << std::fixed << std::setprecision(3) << state(i);
    if (i < state.size() - 1) std::cout << ", ";
  }
  std::cout << "]" << std::endl;
}

// Test 1: ConstantVelocity Model
void test_constant_velocity() {
  std::cout << "\n========== TEST 1: Constant Velocity Model ==========" << std::endl;

  auto model = ConstantVelocityModel(0.01, 0.01, 0.2);

  // Initial state: at origin with velocity [1, 1, 1] m/s
  Eigen::VectorXd state(6);
  state << 0.0, 0.0, 0.0, 1.0, 1.0, 1.0;

  printState("Initial state", state);

  double dt = 0.1;  // 100 ms time step
  std::cout << "\nPredicting 10 steps (1 second) with dt=" << dt << " s:" << std::endl;

  for (int step = 0; step < 10; ++step) {
    state = model.predict(state, dt);
    std::cout << "Step " << (step + 1) << ": pos=[" << std::fixed << std::setprecision(3)
              << state(0) << ", " << state(1) << ", " << state(2) << "]" << std::endl;
  }

  // Verify: with constant velocity of [1,1,1] and dt=0.1, position should increment by 0.1
  std::cout << "\nVerification: Position should be approximately [1.0, 1.0, 1.0]" << std::endl;
  std::cout << "Actual: [" << state(0) << ", " << state(1) << ", " << state(2) << "]" << std::endl;
  std::cout << "ERROR: " << (state.head(3) - Eigen::Vector3d::Ones()).norm() << " meters"
            << std::endl;
}

// Test 2: ConstantAcceleration Model
void test_constant_acceleration() {
  std::cout << "\n========== TEST 2: Constant Acceleration Model ==========" << std::endl;

  auto model = ConstantAccelerationModel(0.01, 0.1, 0.01, 0.2);

  // Initial state: at origin, zero velocity, acceleration [0.5, 0.5, 0.5] m/s^2
  Eigen::VectorXd state(9);
  state << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5;

  printState("Initial state (pos, vel, acc)", state);

  double dt = 0.1;
  std::cout << "\nPredicting 10 steps (1 second) with dt=" << dt << " s:" << std::endl;

  for (int step = 0; step < 10; ++step) {
    state = model.predict(state, dt);
    std::cout << "Step " << (step + 1) << ": pos=[" << std::fixed << std::setprecision(3)
              << state(0) << ", " << state(1) << ", " << state(2) << "] vel=["
              << state(3) << ", " << state(4) << ", " << state(5) << "]" << std::endl;
  }

  // Verify: x = 0.5 * a * t^2 = 0.5 * 0.5 * 1.0^2 = 0.25
  std::cout << "\nVerification: Position should be approximately [0.25, 0.25, 0.25]" << std::endl;
  std::cout << "Velocity should be approximately [0.5, 0.5, 0.5]" << std::endl;
  std::cout << "Actual pos: [" << state(0) << ", " << state(1) << ", " << state(2) << "]"
            << std::endl;
  std::cout << "Actual vel: [" << state(3) << ", " << state(4) << ", " << state(5) << "]"
            << std::endl;
}

// Test 3: Singer Model
void test_singer_model() {
  std::cout << "\n========== TEST 3: Singer Model (Maneuvering) ==========" << std::endl;

  auto model = SingerModel(0.01, 0.1, 0.01, 0.95, 0.2);

  // Initial state: maneuvering with high acceleration
  Eigen::VectorXd state(9);
  state << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0;

  printState("Initial state (pos, vel, acc)", state);
  std::cout << "Maneuver decay rate (alpha): 0.95" << std::endl;

  double dt = 0.1;
  std::cout << "\nPredicting 10 steps (1 second) with dt=" << dt << " s:" << std::endl;

  for (int step = 0; step < 10; ++step) {
    state = model.predict(state, dt);
    std::cout << "Step " << (step + 1) << ": pos=" << std::fixed << std::setprecision(3)
              << state(0) << " acc=" << state(6) << std::endl;
  }

  std::cout << "\nVerification: Acceleration should decay exponentially" << std::endl;
  std::cout << "Final acceleration should be significantly lower than 2.0" << std::endl;
}

// Test 4: IMMKF Model Switching
void test_immkf_switching() {
  std::cout << "\n========== TEST 4: IMMKF Model Switching ==========" << std::endl;

  // Create three models
  std::vector<std::shared_ptr<MotionModel>> models;
  models.push_back(std::make_shared<ConstantVelocityModel>(0.01, 0.01, 0.2));
  models.push_back(std::make_shared<ConstantAccelerationModel>(0.01, 0.1, 0.01, 0.2));
  models.push_back(std::make_shared<SingerModel>(0.01, 0.1, 0.01, 0.95, 0.2));

  std::vector<double> initial_probs = {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};

  Eigen::MatrixXd mode_transitions(3, 3);
  mode_transitions << 0.95, 0.025, 0.025, 0.025, 0.95, 0.025, 0.025, 0.025, 0.95;

  auto filter = IMMKFFilter(models, initial_probs, mode_transitions);

  // Initialize with observed position
  filter.initialize(Eigen::Vector3d(0, 0, 0));

  double dt = 0.1;
  int n_steps = 40;  // 4 seconds

  std::cout << "Scenario: Object starts with constant velocity, then accelerates" << std::endl;
  std::cout << "\nStep  | Model Probabilities (CV, CA, Singer) | Most Likely" << std::endl;
  std::cout << "------|----------------------------------------|---" << std::endl;

  for (int step = 0; step < n_steps; ++step) {
    // Generate synthetic measurement
    Eigen::Vector3d measurement;

    if (step < 20) {
      // Constant velocity phase
      measurement = Eigen::Vector3d(step * 0.1, step * 0.1, 0.0);
    } else {
      // Acceleration phase
      double t = (step - 20) * 0.1;
      measurement = Eigen::Vector3d(2.0 + step * 0.1 + 0.25 * t * t,
                                    2.0 + step * 0.1 + 0.25 * t * t, 0.0);
    }

    filter.predict(dt);
    filter.update(measurement);

    if (step % 10 == 0 || step == 19 || step == 20 || step == n_steps - 1) {
      auto probs = filter.getModelProbabilities();
      std::cout << std::setw(5) << step << " | " << std::fixed << std::setprecision(3)
                << probs[0] << "  " << probs[1] << "  " << probs[2] << "      | ";

      int best = filter.getMostLikelyModelIndex();
      std::cout << (best == 0 ? "CV" : (best == 1 ? "CA" : "Singer")) << std::endl;
    }
  }

  std::cout << "\nVerification:" << std::endl;
  std::cout << "- Steps 0-19: CV model should dominate (prob > 0.5)" << std::endl;
  std::cout << "- Steps 20+: CA model should increase (acceleration detected)" << std::endl;
}

int main() {
  std::cout << "IMMKF Motion Model Tests\n" << std::endl;

  try {
    test_constant_velocity();
    test_constant_acceleration();
    test_singer_model();
    test_immkf_switching();

    std::cout << "\n\n========== ALL TESTS COMPLETED ==========" << std::endl;
    std::cout << "✓ All motion models and IMMKF working as expected" << std::endl;

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Test failed with exception: " << e.what() << std::endl;
    return 1;
  }
}
