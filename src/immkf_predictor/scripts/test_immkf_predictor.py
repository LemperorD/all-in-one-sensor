#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Full IMMKF Demonstration
=====================================

Features:
- CV model
- CA model
- Singer model
- CT model
- IMM interaction
- 3D UAV trajectory
- Noisy measurements
- Model probability evolution
- GIF animation generation

Author: ChatGPT
"""

import os
import numpy as np
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation, PillowWriter


# =========================================================
# Result directory
# =========================================================

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULT_DIR = os.path.join(SCRIPT_DIR, "results")

os.makedirs(RESULT_DIR, exist_ok=True)


# =========================================================
# Basic Linear Kalman Filter
# =========================================================

class KalmanFilter:

    def __init__(self, dim_x, dim_z):

        self.dim_x = dim_x
        self.dim_z = dim_z

        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x)

        self.F = np.eye(dim_x)

        self.H = np.zeros((dim_z, dim_x))

        self.Q = np.eye(dim_x) * 0.01
        self.R = np.eye(dim_z) * 0.5

    def predict(self):

        self.x = self.F @ self.x

        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):

        y = z - self.H @ self.x

        S = self.H @ self.P @ self.H.T + self.R

        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y

        I = np.eye(self.dim_x)

        self.P = (I - K @ self.H) @ self.P

        # likelihood
        det_S = np.linalg.det(S)

        if det_S < 1e-6:
            det_S = 1e-6

        exponent = -0.5 * y.T @ np.linalg.inv(S) @ y

        likelihood = np.exp(exponent) / np.sqrt(
            ((2 * np.pi) ** self.dim_z) * det_S
        )

        return likelihood


# =========================================================
# CV Model
# =========================================================

class CVModel(KalmanFilter):

    def __init__(self, dt):

        super().__init__(6, 3)

        self.dt = dt

        self.F = np.eye(6)

        for i in range(3):
            self.F[i, i + 3] = dt

        self.H[0, 0] = 1
        self.H[1, 1] = 1
        self.H[2, 2] = 1

        self.Q *= 0.05


# =========================================================
# CA Model
# =========================================================

class CAModel(KalmanFilter):

    def __init__(self, dt):

        super().__init__(9, 3)

        self.dt = dt

        self.F = np.eye(9)

        for i in range(3):

            self.F[i, i + 3] = dt
            self.F[i, i + 6] = 0.5 * dt * dt

            self.F[i + 3, i + 6] = dt

        self.H[0, 0] = 1
        self.H[1, 1] = 1
        self.H[2, 2] = 1

        self.Q *= 0.1


# =========================================================
# Singer Model
# =========================================================

class SingerModel(KalmanFilter):

    def __init__(self, dt, alpha=0.95):

        super().__init__(9, 3)

        self.dt = dt
        self.alpha = alpha

        e = np.exp(-alpha * dt)

        self.F = np.eye(9)

        for i in range(3):

            self.F[i, i + 3] = dt
            self.F[i + 3, i + 6] = (1 - e)

            self.F[i + 6, i + 6] = e

        self.H[0, 0] = 1
        self.H[1, 1] = 1
        self.H[2, 2] = 1

        self.Q *= 0.15


# =========================================================
# CT Model
# =========================================================

class CTModel(KalmanFilter):

    def __init__(self, dt):

        super().__init__(7, 3)

        self.dt = dt

        self.H[0, 0] = 1
        self.H[1, 1] = 1
        self.H[2, 2] = 1

        self.Q *= 0.1

    def predict(self):

        x, y, z, v, yaw, yaw_rate, vz = self.x

        dt = self.dt

        if abs(yaw_rate) < 1e-4:

            x += v * np.cos(yaw) * dt
            y += v * np.sin(yaw) * dt

        else:

            x += (
                v / yaw_rate
                * (
                    np.sin(yaw + yaw_rate * dt)
                    - np.sin(yaw)
                )
            )

            y += (
                v / yaw_rate
                * (
                    -np.cos(yaw + yaw_rate * dt)
                    + np.cos(yaw)
                )
            )

        z += vz * dt

        yaw += yaw_rate * dt

        self.x = np.array([
            x,
            y,
            z,
            v,
            yaw,
            yaw_rate,
            vz
        ])

        self.P = self.P + self.Q


# =========================================================
# IMM Filter
# =========================================================

class IMMKF:

    def __init__(self, models, transition_matrix):

        self.models = models

        self.M = len(models)

        self.transition_matrix = transition_matrix

        self.mu = np.ones(self.M) / self.M

    def predict(self):

        for model in self.models:
            model.predict()

    def update(self, z):

        likelihoods = np.zeros(self.M)

        for i, model in enumerate(self.models):

            likelihoods[i] = model.update(z)

        # model probability update
        self.mu = self.mu * likelihoods

        self.mu /= np.sum(self.mu)

    def fused_position(self):

        pos = np.zeros(3)

        for i, model in enumerate(self.models):

            pos += self.mu[i] * model.x[0:3]

        return pos


# =========================================================
# Generate UAV trajectory
# =========================================================

def generate_uav_trajectory():

    gt = []

    dt = 0.1

    x = 0
    y = 0
    z = 0

    vx = 1
    vy = 0
    vz = 0.1

    # =====================================================
    # Phase 1 : CV
    # =====================================================

    for _ in range(50):

        x += vx * dt
        y += vy * dt
        z += vz * dt

        gt.append([x, y, z])

    # =====================================================
    # Phase 2 : Acceleration
    # =====================================================

    ax = 0.05

    for _ in range(50):

        vx += ax * dt

        x += vx * dt
        y += vy * dt
        z += vz * dt

        gt.append([x, y, z])

    # =====================================================
    # Phase 3 : Turning
    # =====================================================

    yaw = 0

    speed = 2.0

    yaw_rate = np.deg2rad(20)

    for _ in range(100):

        yaw += yaw_rate * dt

        x += speed * np.cos(yaw) * dt
        y += speed * np.sin(yaw) * dt
        z += vz * dt

        gt.append([x, y, z])

    return np.array(gt)


# =========================================================
# Main
# =========================================================

def main():

    dt = 0.1

    gt = generate_uav_trajectory()

    noise_std = 0.5

    measurements = (
        gt
        + np.random.randn(*gt.shape) * noise_std
    )

    # =====================================================
    # Create models
    # =====================================================

    cv = CVModel(dt)

    ca = CAModel(dt)

    singer = SingerModel(dt)

    ct = CTModel(dt)

    models = [cv, ca, singer, ct]

    # initialize
    for model in models:

        model.x[0:3] = measurements[0]

    # CT initialization
    ct.x[3] = 1.0
    ct.x[4] = 0.0
    ct.x[5] = np.deg2rad(10)
    ct.x[6] = 0.1

    # =====================================================
    # IMM
    # =====================================================

    transition_matrix = np.array([

        [0.90, 0.03, 0.03, 0.04],
        [0.03, 0.90, 0.03, 0.04],
        [0.03, 0.03, 0.90, 0.04],
        [0.03, 0.03, 0.04, 0.90]

    ])

    imm = IMMKF(models, transition_matrix)

    fused_positions = []

    model_probs = []

    # =====================================================
    # Filtering loop
    # =====================================================

    for z in measurements:

        imm.predict()

        imm.update(z)

        fused_positions.append(
            imm.fused_position()
        )

        model_probs.append(
            imm.mu.copy()
        )

    fused_positions = np.array(fused_positions)

    model_probs = np.array(model_probs)

    # =====================================================
    # Animation
    # =====================================================

    fig = plt.figure(figsize=(14, 7))

    ax3d = fig.add_subplot(121, projection='3d')

    ax_prob = fig.add_subplot(122)

    def update(frame):

        ax3d.clear()
        ax_prob.clear()

        # =============================================
        # 3D Trajectory
        # =============================================

        ax3d.plot(
            gt[:frame, 0],
            gt[:frame, 1],
            gt[:frame, 2],
            linewidth=3,
            label='Ground Truth'
        )

        ax3d.scatter(
            measurements[:frame, 0],
            measurements[:frame, 1],
            measurements[:frame, 2],
            s=5,
            alpha=0.4,
            label='Measurements'
        )

        ax3d.plot(
            fused_positions[:frame, 0],
            fused_positions[:frame, 1],
            fused_positions[:frame, 2],
            linewidth=2,
            label='IMMKF Prediction'
        )

        ax3d.set_title("3D UAV Trajectory")

        ax3d.set_xlabel("X")
        ax3d.set_ylabel("Y")
        ax3d.set_zlabel("Z")

        ax3d.legend()

        # =============================================
        # Model probabilities
        # =============================================

        t = np.arange(frame)

        labels = [
            "CV",
            "CA",
            "Singer",
            "CT"
        ]

        for i in range(4):

            ax_prob.plot(
                t,
                model_probs[:frame, i],
                label=labels[i]
            )

        ax_prob.set_ylim([0, 1])

        ax_prob.set_title("Model Probabilities")

        ax_prob.set_xlabel("Frame")
        ax_prob.set_ylabel("Probability")

        ax_prob.legend()

        ax_prob.grid(True)

    anim = FuncAnimation(

        fig,
        update,
        frames=len(gt),
        interval=50

    )

    # =====================================================
    # Save GIF
    # =====================================================

    gif_path = os.path.join(
        RESULT_DIR,
        "immkf_demo.gif"
    )

    print("\nSaving GIF animation...")

    anim.save(
        gif_path,
        writer=PillowWriter(fps=20)
    )

    print("\n======================================")
    print("DONE")
    print("======================================")

    print(f"\nGIF saved to:\n{gif_path}")

    # =====================================================
    # Save final figure
    # =====================================================

    png_path = os.path.join(
        RESULT_DIR,
        "immkf_final.png"
    )

    plt.savefig(
        png_path,
        dpi=200
    )

    print(f"\nPNG saved to:\n{png_path}")

    plt.show()


if __name__ == "__main__":

    main()