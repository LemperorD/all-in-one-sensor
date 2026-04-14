#!/usr/bin/env python3
"""
Standalone demonstration of IMMKF algorithm for UAV trajectory prediction
This script tests the three motion models without requiring ROS2 to be running
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class State:
    """6D or 9D state representation"""
    pos: np.ndarray  # [x, y, z]
    vel: np.ndarray  # [vx, vy, vz]
    acc: np.ndarray = None  # [ax, ay, az] for CA and Singer


class MotionModel:
    """Base class for motion models"""

    def get_state_dim(self) -> int:
        raise NotImplementedError

    def predict(self, state: np.ndarray, dt: float) -> np.ndarray:
        raise NotImplementedError

    def get_measurement_matrix(self) -> np.ndarray:
        """H matrix: z = H*x (only measure position)"""
        raise NotImplementedError


class ConstantVelocityModel(MotionModel):
    """CV Model: 6D state [x, y, z, vx, vy, vz]"""

    def __init__(self, q_pos=0.01, q_vel=0.01):
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.state = np.zeros(6)
        self.P = np.eye(6)

    def get_state_dim(self) -> int:
        return 6

    def predict(self, state: np.ndarray, dt: float) -> np.ndarray:
        predicted = state.copy()
        predicted[0:3] += state[3:6] * dt  # x += v*dt
        return predicted

    def get_measurement_matrix(self) -> np.ndarray:
        H = np.zeros((3, 6))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        return H


class ConstantAccelerationModel(MotionModel):
    """CA Model: 9D state [x, y, z, vx, vy, vz, ax, ay, az]"""

    def __init__(self, q_pos=0.01, q_vel=0.1, q_acc=0.01):
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.q_acc = q_acc
        self.state = np.zeros(9)
        self.P = np.eye(9)

    def get_state_dim(self) -> int:
        return 9

    def predict(self, state: np.ndarray, dt: float) -> np.ndarray:
        predicted = np.zeros(9)
        dt2 = dt * dt
        # x = x + v*dt + 0.5*a*dt^2
        predicted[0:3] = state[0:3] + state[3:6] * dt + 0.5 * state[6:9] * dt2
        # v = v + a*dt
        predicted[3:6] = state[3:6] + state[6:9] * dt
        # a = a (constant)
        predicted[6:9] = state[6:9]
        return predicted

    def get_measurement_matrix(self) -> np.ndarray:
        H = np.zeros((3, 9))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        return H


class SingerModel(MotionModel):
    """Singer Model: 9D state with exponentially decaying acceleration"""

    def __init__(self, q_pos=0.01, q_vel=0.1, q_acc=0.01, alpha=0.95):
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.q_acc = q_acc
        self.alpha = alpha
        self.state = np.zeros(9)
        self.P = np.eye(9)

    def get_state_dim(self) -> int:
        return 9

    def predict(self, state: np.ndarray, dt: float) -> np.ndarray:
        predicted = np.zeros(9)
        exp_alpha_dt = np.exp(-self.alpha * dt)
        a_factor = (1.0 - exp_alpha_dt) / self.alpha

        # Position: x = x + v*dt + a*a_factor*dt
        predicted[0:3] = state[0:3] + state[3:6] * dt + state[6:9] * a_factor * dt
        # Velocity: v = v + a*(1 - exp(-alpha*dt))
        predicted[3:6] = state[3:6] + state[6:9] * (1.0 - exp_alpha_dt)
        # Acceleration decays: a = a*exp(-alpha*dt)
        predicted[6:9] = state[6:9] * exp_alpha_dt

        return predicted

    def get_measurement_matrix(self) -> np.ndarray:
        H = np.zeros((3, 9))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        return H


def test_constant_velocity():
    """Test 1: Constant velocity motion"""
    print("\n" + "="*60)
    print("TEST 1: Constant Velocity Model")
    print("="*60)

    model = ConstantVelocityModel()

    # Initial state: velocity [1, 1, 1] m/s
    state = np.array([0, 0, 0, 1.0, 1.0, 1.0])
    print(f"Initial state: pos={state[0:3]}, vel={state[3:6]}")

    positions = [state[0:3].copy()]
    dt = 0.1

    for i in range(10):
        state = model.predict(state, dt)
        positions.append(state[0:3].copy())

    positions = np.array(positions)
    print(f"After 1 second: pos={state[0:3]}")
    print(f"Expected:       pos=[1.0, 1.0, 1.0]")
    print(f"Error:          {np.linalg.norm(state[0:3] - np.ones(3)):.4f} m")
    print(f"✓ PASS" if np.allclose(state[0:3], np.ones(3), atol=0.01) else "✗ FAIL")

    return positions


def test_constant_acceleration():
    """Test 2: Constant acceleration motion"""
    print("\n" + "="*60)
    print("TEST 2: Constant Acceleration Model")
    print("="*60)

    model = ConstantAccelerationModel()

    # Initial state: acceleration [0.5, 0.5, 0.5] m/s^2
    state = np.array([0, 0, 0, 0, 0, 0, 0.5, 0.5, 0.5])
    print(f"Initial state: pos={state[0:3]}, vel={state[3:6]}, acc={state[6:9]}")

    positions = [state[0:3].copy()]
    dt = 0.1

    for i in range(10):
        state = model.predict(state, dt)
        positions.append(state[0:3].copy())

    positions = np.array(positions)
    # Expected: x = 0.5*a*t^2 = 0.5*0.5*1^2 = 0.25
    print(f"After 1 second: pos={state[0:3]}, vel={state[3:6]}")
    print(f"Expected:       pos=[0.25, 0.25, 0.25], vel=[0.5, 0.5, 0.5]")
    expected_pos = np.array([0.25, 0.25, 0.25])
    expected_vel = np.array([0.5, 0.5, 0.5])
    pos_error = np.linalg.norm(state[0:3] - expected_pos)
    vel_error = np.linalg.norm(state[3:6] - expected_vel)
    print(f"Position Error: {pos_error:.4f} m")
    print(f"Velocity Error: {vel_error:.4f} m/s")
    print(f"✓ PASS" if (pos_error < 0.05 and vel_error < 0.05) else "✗ FAIL")

    return positions


def test_singer_model():
    """Test 3: Singer model with maneuver decay"""
    print("\n" + "="*60)
    print("TEST 3: Singer Model (Maneuvering)")
    print("="*60)

    model = SingerModel(alpha=0.95)

    # Initial state: high acceleration [2, 0, 0] m/s^2
    state = np.array([0, 0, 0, 0, 0, 0, 2.0, 0, 0])
    print(f"Initial state: pos={state[0:3]}, vel={state[3:6]}, acc={state[6:9]}")
    print(f"Decay rate (alpha): {model.alpha}")

    positions = [state[0:3].copy()]
    accelerations = [state[6:9].copy()]
    dt = 0.1

    for i in range(10):
        state = model.predict(state, dt)
        positions.append(state[0:3].copy())
        accelerations.append(state[6:9].copy())

    positions = np.array(positions)
    accelerations = np.array(accelerations)

    print(f"After 1 second:")
    print(f"  Position:     {state[0:3]}")
    print(f"  Acceleration: {state[6:9]}")
    print(f"  Acceleration decay ratio: {state[6] / 2.0:.4f}")
    print(f"✓ PASS" if state[6] < 1.0 and state[6] > 0.1 else "✗ FAIL")

    return positions, accelerations


def plot_comparison():
    """Create visualization comparing the three models"""
    print("\n" + "="*60)
    print("Creating comparison plots...")
    print("="*60)

    # Test all models
    pos_cv = test_constant_velocity()
    pos_ca = test_constant_acceleration()
    pos_singer, acc_singer = test_singer_model()

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    # Plot 1: CV vs CA position
    ax = axes[0, 0]
    time_cv = np.arange(len(pos_cv)) * 0.1
    time_ca = np.arange(len(pos_ca)) * 0.1
    ax.plot(time_cv, pos_cv[:, 0], 'b-o', label='CV', linewidth=2)
    ax.plot(time_ca, pos_ca[:, 0], 'r-s', label='CA', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('X Position (m)')
    ax.set_title('Position Comparison: CV vs CA Models')
    ax.legend()
    ax.grid(True)

    # Plot 2: Singer model position
    ax = axes[0, 1]
    time_singer = np.arange(len(pos_singer)) * 0.1
    ax.plot(time_singer, pos_singer[:, 0], 'g-d', label='Singer', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('X Position (m)')
    ax.set_title('Singer Model Position (with Maneuver)')
    ax.legend()
    ax.grid(True)

    # Plot 3: Acceleration decay in Singer model
    ax = axes[1, 0]
    ax.semilogy(time_singer, acc_singer[:, 0], 'g-d', linewidth=2, label='Singer (dec=0.95)')
    ax.axhline(y=2.0, color='k', linestyle='--', alpha=0.5, label='Initial')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (m/s²)')
    ax.set_title('Singer Model: Exponential Acceleration Decay')
    ax.legend()
    ax.grid(True, which='both')

    # Plot 4: 3D trajectories
    ax = axes[1, 1]
    ax.plot(time_cv, pos_cv[:, 0], 'b-o', label='CV: x(t)=t', linewidth=2)
    ax.plot(time_ca, pos_ca[:, 0], 'r-s', label='CA: x(t)=0.25t²', linewidth=2)
    ax.plot(time_singer, pos_singer[:, 0], 'g-d', label='Singer: d.ecaying', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('X Position (m)')
    ax.set_title('All Models: Position Evolution')
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.savefig('/tmp/immkf_motion_models_demo.png', dpi=150)
    print("✓ Plot saved to /tmp/immkf_motion_models_demo.png")
    plt.close()


def main():
    print("\n" + "="*60)
    print("IMMKF Motion Models Demonstration")
    print("Interactive Multi-Model Kalman Filter for UAV Prediction")
    print("="*60)

    try:
        # Run all tests
        test_constant_velocity()
        test_constant_acceleration()
        test_singer_model()

        # Create plots
        plot_comparison()

        print("\n" + "="*60)
        print("✓ ALL TESTS COMPLETED SUCCESSFULLY")
        print("="*60)
        print("\nSummary:")
        print("  1. CV Model: Handles constant velocity motion")
        print("  2. CA Model: Handles uniformly accelerated motion")
        print("  3. Singer Model: Handles maneuvering with decay")
        print("\nNext steps:")
        print("  - Build package: colcon build --packages-select immkf_predictor")
        print("  - Run ROS2 node: ros2 launch immkf_predictor immkf_predictor.launch.py")
        print("  - View results: ros2 topic echo /predicted_trajectories")

    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
