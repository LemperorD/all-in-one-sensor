#!/usr/bin/env python3
"""
Standalone demonstration of IMMKF algorithm for UAV trajectory prediction
This script tests multiple motion models without requiring ROS2 to be running
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple


# ==============================
# Create results directory
# ==============================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_DIR = os.path.join(SCRIPT_DIR, "results")
os.makedirs(RESULTS_DIR, exist_ok=True)


@dataclass
class State:
    """6D / 9D / turning model state representation"""
    pos: np.ndarray
    vel: np.ndarray
    acc: np.ndarray = None


class MotionModel:
    """Base class for motion models"""

    def get_state_dim(self) -> int:
        raise NotImplementedError

    def predict(self, state: np.ndarray, dt: float) -> np.ndarray:
        raise NotImplementedError

    def get_measurement_matrix(self) -> np.ndarray:
        raise NotImplementedError


# =========================================================
# Constant Velocity Model
# =========================================================
class ConstantVelocityModel(MotionModel):
    """
    CV Model:
    [x, y, z, vx, vy, vz]
    """

    def __init__(self, q_pos=0.01, q_vel=0.01):
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.state = np.zeros(6)
        self.P = np.eye(6)

    def get_state_dim(self) -> int:
        return 6

    def predict(self, state: np.ndarray, dt: float) -> np.ndarray:
        predicted = state.copy()
        predicted[0:3] += state[3:6] * dt
        return predicted

    def get_measurement_matrix(self) -> np.ndarray:
        H = np.zeros((3, 6))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        return H


# =========================================================
# Constant Acceleration Model
# =========================================================
class ConstantAccelerationModel(MotionModel):
    """
    CA Model:
    [x, y, z, vx, vy, vz, ax, ay, az]
    """

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

        predicted[0:3] = (
            state[0:3]
            + state[3:6] * dt
            + 0.5 * state[6:9] * dt2
        )

        predicted[3:6] = (
            state[3:6]
            + state[6:9] * dt
        )

        predicted[6:9] = state[6:9]

        return predicted

    def get_measurement_matrix(self) -> np.ndarray:
        H = np.zeros((3, 9))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        return H


# =========================================================
# Singer Model
# =========================================================
class SingerModel(MotionModel):
    """
    Singer Model:
    exponentially decaying acceleration
    """

    def __init__(self, q_pos=0.01, q_vel=0.1,
                 q_acc=0.01, alpha=0.95):

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

        predicted[0:3] = (
            state[0:3]
            + state[3:6] * dt
            + state[6:9] * a_factor * dt
        )

        predicted[3:6] = (
            state[3:6]
            + state[6:9] * (1.0 - exp_alpha_dt)
        )

        predicted[6:9] = (
            state[6:9] * exp_alpha_dt
        )

        return predicted

    def get_measurement_matrix(self) -> np.ndarray:
        H = np.zeros((3, 9))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        return H


# =========================================================
# Constant Turn Rate Model
# =========================================================
class ConstantTurnRateModel(MotionModel):
    """
    CTRV-like planar turn model

    State:
    [x, y, z, v, yaw, yaw_rate, vz]

    x,y        : planar position
    z          : altitude
    v          : forward speed
    yaw        : heading angle
    yaw_rate   : angular velocity
    vz         : vertical velocity
    """

    def __init__(self):
        self.state = np.zeros(7)
        self.P = np.eye(7)

    def get_state_dim(self):
        return 7

    def predict(self, state, dt):

        x, y, z, v, yaw, yaw_rate, vz = state

        predicted = np.zeros(7)

        if abs(yaw_rate) < 1e-5:
            predicted[0] = x + v * np.cos(yaw) * dt
            predicted[1] = y + v * np.sin(yaw) * dt
        else:
            predicted[0] = (
                x
                + v / yaw_rate
                * (
                    np.sin(yaw + yaw_rate * dt)
                    - np.sin(yaw)
                )
            )

            predicted[1] = (
                y
                + v / yaw_rate
                * (
                    -np.cos(yaw + yaw_rate * dt)
                    + np.cos(yaw)
                )
            )

        predicted[2] = z + vz * dt

        predicted[3] = v
        predicted[4] = yaw + yaw_rate * dt
        predicted[5] = yaw_rate
        predicted[6] = vz

        return predicted

    def get_measurement_matrix(self):

        H = np.zeros((3, 7))

        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0

        return H


# =========================================================
# TESTS
# =========================================================
def test_constant_velocity():

    print("\n" + "=" * 60)
    print("TEST 1: Constant Velocity Model")
    print("=" * 60)

    model = ConstantVelocityModel()

    state = np.array([0, 0, 0, 1.0, 1.0, 1.0])

    positions = [state[0:3].copy()]
    dt = 0.1

    for _ in range(10):
        state = model.predict(state, dt)
        positions.append(state[0:3].copy())

    positions = np.array(positions)

    print(f"After 1 second: {state[0:3]}")

    return positions


def test_constant_acceleration():

    print("\n" + "=" * 60)
    print("TEST 2: Constant Acceleration Model")
    print("=" * 60)

    model = ConstantAccelerationModel()

    state = np.array([
        0, 0, 0,
        0, 0, 0,
        0.5, 0.5, 0.5
    ])

    positions = [state[0:3].copy()]
    dt = 0.1

    for _ in range(10):
        state = model.predict(state, dt)
        positions.append(state[0:3].copy())

    positions = np.array(positions)

    print(f"After 1 second: {state[0:3]}")

    return positions


def test_singer_model():

    print("\n" + "=" * 60)
    print("TEST 3: Singer Model")
    print("=" * 60)

    model = SingerModel(alpha=0.95)

    state = np.array([
        0, 0, 0,
        0, 0, 0,
        2.0, 0, 0
    ])

    positions = [state[0:3].copy()]
    accelerations = [state[6:9].copy()]

    dt = 0.1

    for _ in range(10):
        state = model.predict(state, dt)
        positions.append(state[0:3].copy())
        accelerations.append(state[6:9].copy())

    positions = np.array(positions)
    accelerations = np.array(accelerations)

    print(f"Final acceleration: {state[6:9]}")

    return positions, accelerations


def test_constant_turn_rate():

    print("\n" + "=" * 60)
    print("TEST 4: Constant Turn Rate Model")
    print("=" * 60)

    model = ConstantTurnRateModel()

    # x,y,z,v,yaw,yaw_rate,vz
    state = np.array([
        0.0,
        0.0,
        0.0,
        2.0,
        0.0,
        np.deg2rad(30.0),
        0.0
    ])

    positions = [state[0:3].copy()]
    yaws = [state[4]]

    dt = 0.1

    for _ in range(50):

        state = model.predict(state, dt)

        positions.append(state[0:3].copy())
        yaws.append(state[4])

    positions = np.array(positions)
    yaws = np.array(yaws)

    print(f"Final position: {state[0:3]}")
    print(f"Final yaw(deg): {np.rad2deg(state[4]):.2f}")

    return positions, yaws


# =========================================================
# PLOT
# =========================================================
def plot_comparison():

    print("\n" + "=" * 60)
    print("Creating comparison plots...")
    print("=" * 60)

    pos_cv = test_constant_velocity()
    pos_ca = test_constant_acceleration()
    pos_singer, acc_singer = test_singer_model()
    pos_turn, yaws = test_constant_turn_rate()

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # ---------------------------------------
    # CV vs CA
    # ---------------------------------------
    ax = axes[0, 0]

    t_cv = np.arange(len(pos_cv)) * 0.1
    t_ca = np.arange(len(pos_ca)) * 0.1

    ax.plot(t_cv, pos_cv[:, 0], label='CV')
    ax.plot(t_ca, pos_ca[:, 0], label='CA')

    ax.set_title("CV vs CA")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("X Position")
    ax.grid(True)
    ax.legend()

    # ---------------------------------------
    # Singer
    # ---------------------------------------
    ax = axes[0, 1]

    t_singer = np.arange(len(pos_singer)) * 0.1

    ax.plot(t_singer, pos_singer[:, 0], label='Singer')

    ax.set_title("Singer Position")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("X Position")
    ax.grid(True)
    ax.legend()

    # ---------------------------------------
    # Singer decay
    # ---------------------------------------
    ax = axes[1, 0]

    ax.semilogy(
        t_singer,
        acc_singer[:, 0],
        label='Acceleration Decay'
    )

    ax.set_title("Singer Acceleration Decay")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration")
    ax.grid(True)
    ax.legend()

    # ---------------------------------------
    # Turning trajectory
    # ---------------------------------------
    ax = axes[1, 1]

    ax.plot(
        pos_turn[:, 0],
        pos_turn[:, 1],
        '-o',
        label='Turn Motion'
    )

    ax.set_aspect('equal')

    ax.set_title("Constant Angular Velocity Motion")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True)
    ax.legend()

    plt.tight_layout()

    save_path = os.path.join(
        RESULTS_DIR,
        "immkf_motion_models_demo.png"
    )

    plt.savefig(save_path, dpi=150)

    print(f"✓ Plot saved to:")
    print(save_path)

    plt.close()


# =========================================================
# MAIN
# =========================================================
def main():

    print("\n" + "=" * 60)
    print("IMMKF Motion Models Demonstration")
    print("=" * 60)

    try:

        test_constant_velocity()
        test_constant_acceleration()
        test_singer_model()
        test_constant_turn_rate()

        plot_comparison()

        print("\n" + "=" * 60)
        print("✓ ALL TESTS COMPLETED SUCCESSFULLY")
        print("=" * 60)

        print("\nResults saved to:")
        print(RESULTS_DIR)

    except Exception as e:

        print(f"\n✗ ERROR: {e}")

        import traceback
        traceback.print_exc()

        return 1

    return 0


if __name__ == "__main__":
    exit(main())