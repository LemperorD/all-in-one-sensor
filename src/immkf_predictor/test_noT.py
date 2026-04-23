#!/usr/bin/env python3
"""
2D IMM-KF WITHOUT CT MODEL (for comparison)
Models: CV / CA / Singer
- Same trajectory
- Dual animation (trajectory + error)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

# ===================== Ground Truth =====================
def generate_ground_truth(T=30.0, dt=0.1):
    t = np.arange(0, T, dt)
    pos = []

    for ti in t:
        s = 0.5 * (1 - np.cos(np.pi * ti / T))
        x = 10 * s * np.cos(0.6 * ti)
        y = 10 * s * np.sin(0.6 * ti)
        pos.append([x, y])

    return t, np.array(pos)


def add_noise(traj, noise_std=0.15):
    return traj + np.random.randn(*traj.shape) * noise_std


# ===================== KF =====================
class KF:
    def __init__(self, dim_x, dim_z):
        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x)
        self.F = np.eye(dim_x)
        self.Q = np.eye(dim_x) * 0.01
        self.H = np.zeros((dim_z, dim_x))
        self.R = np.eye(dim_z) * 0.2

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(len(self.x)) - K @ self.H) @ self.P


# ===================== Models =====================
def create_cv(dt):
    kf = KF(4, 2)
    F = np.eye(4)
    F[0, 2] = dt
    F[1, 3] = dt
    kf.F = F
    kf.H[:, :2] = np.eye(2)
    return kf


def create_ca(dt):
    kf = KF(6, 2)
    F = np.eye(6)
    for i in range(2):
        F[i, i+2] = dt
        F[i, i+4] = 0.5 * dt**2
        F[i+2, i+4] = dt
    kf.F = F
    kf.H[:, :2] = np.eye(2)
    return kf


def create_singer(dt, alpha=0.8):
    kf = KF(6, 2)
    F = np.eye(6)
    exp_a = np.exp(-alpha * dt)
    c1 = (1 - exp_a) / alpha
    c2 = (dt - c1) / alpha

    for i in range(2):
        F[i, i+2] = dt
        F[i, i+4] = c2
        F[i+2, i+4] = c1
        F[i+4, i+4] = exp_a

    kf.F = F
    kf.H[:, :2] = np.eye(2)
    return kf


# ===================== IMM =====================
class IMM:
    def __init__(self, models):
        self.models = models
        self.mu = np.ones(len(models)) / len(models)

    def step(self, z):
        likelihoods = []
        states = []

        for model in self.models:
            model.predict()
            model.update(z)

            innovation = z - model.H @ model.x
            likelihood = np.exp(-0.5 * np.linalg.norm(innovation))
            likelihoods.append(likelihood)
            states.append(model.x.copy())

        likelihoods = np.array(likelihoods)
        self.mu = self.mu * likelihoods
        self.mu = self.mu / np.sum(self.mu)

        fused = np.zeros(2)
        for i, m in enumerate(self.models):
            fused += self.mu[i] * m.x[:2]

        return fused, states


# ===================== Main =====================
def main():
    dt = 0.1
    t, gt = generate_ground_truth()
    meas = add_noise(gt)

    cv = create_cv(dt)
    ca = create_ca(dt)
    singer = create_singer(dt)

    imm = IMM([cv, ca, singer])

    preds = []
    cv_preds, ca_preds, sg_preds = [], [], []

    for z in meas:
        fused, states = imm.step(z)

        preds.append(fused)
        cv_preds.append(states[0][:2])
        ca_preds.append(states[1][:2])
        sg_preds.append(states[2][:2])

    preds = np.array(preds)
    cv_preds = np.array(cv_preds)
    ca_preds = np.array(ca_preds)
    sg_preds = np.array(sg_preds)

    def err(est): return np.linalg.norm(est - gt, axis=1)

    err_imm = err(preds)
    err_cv = err(cv_preds)
    err_ca = err(ca_preds)
    err_sg = err(sg_preds)

    # ===================== Dual Animation =====================
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 7))

    def update(frame):
        ax1.clear()
        ax2.clear()

        ax1.plot(gt[:frame,0], gt[:frame,1], label='GT')
        ax1.scatter(meas[:frame,0], meas[:frame,1], s=10)
        ax1.plot(cv_preds[:frame,0], cv_preds[:frame,1], '--', label='CV')
        ax1.plot(ca_preds[:frame,0], ca_preds[:frame,1], '--', label='CA')
        ax1.plot(sg_preds[:frame,0], sg_preds[:frame,1], '--', label='Singer')
        ax1.plot(preds[:frame,0], preds[:frame,1], linewidth=2.5, label='IMM')
        ax1.legend(); ax1.grid()

        ax2.plot(t[:frame], err_cv[:frame], '--', linewidth=1, label='CV')
        ax2.plot(t[:frame], err_ca[:frame], '--', linewidth=1, label='CA')
        ax2.plot(t[:frame], err_sg[:frame], '--', linewidth=1, label='Singer')
        ax2.plot(t[:frame], err_imm[:frame], linewidth=2.5, label='IMM')
        ax2.legend(); ax2.grid()

    ani = animation.FuncAnimation(fig, update, frames=len(t), interval=100)
    ani.save('/tmp/immkf.gif', writer='pillow', dpi=200)

    print('Saved dual animation: /tmp/immkf.gif')


if __name__ == '__main__':
    main()
