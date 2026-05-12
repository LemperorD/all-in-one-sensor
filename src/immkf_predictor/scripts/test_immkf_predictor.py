#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULT_DIR = os.path.join(SCRIPT_DIR, "results")
os.makedirs(RESULT_DIR, exist_ok=True)

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
        det_S = max(np.linalg.det(S), 1e-6)
        exponent = -0.5 * y.T @ np.linalg.inv(S) @ y
        return np.exp(exponent) / np.sqrt(((2*np.pi)**self.dim_z) * det_S)

class CVModel(KalmanFilter):
    def __init__(self, dt):
        super().__init__(6, 3)
        self.F = np.eye(6)
        for i in range(3):
            self.F[i, i+3] = dt
            self.H[i, i] = 1
        self.Q *= 0.05

class CAModel(KalmanFilter):
    def __init__(self, dt):
        super().__init__(9, 3)
        self.F = np.eye(9)
        for i in range(3):
            self.F[i, i+3] = dt
            self.F[i, i+6] = 0.5 * dt * dt
            self.F[i+3, i+6] = dt
            self.H[i, i] = 1
        self.Q *= 0.1

class SingerModel(KalmanFilter):
    def __init__(self, dt, alpha=0.95):
        super().__init__(9, 3)
        e = np.exp(-alpha * dt)
        self.F = np.eye(9)
        for i in range(3):
            self.F[i, i+3] = dt
            self.F[i+3, i+6] = (1 - e)
            self.F[i+6, i+6] = e
            self.H[i, i] = 1
        self.Q *= 0.15

class CTModel(KalmanFilter):
    def __init__(self, dt):
        super().__init__(7, 3)
        self.dt = dt
        for i in range(3):
            self.H[i, i] = 1
        self.Q *= 0.1

    def predict(self):
        x, y, z, v, yaw, yaw_rate, vz = self.x
        dt = self.dt
        if abs(yaw_rate) < 1e-4:
            x += v * np.cos(yaw) * dt
            y += v * np.sin(yaw) * dt
        else:
            x += v / yaw_rate * (np.sin(yaw + yaw_rate * dt) - np.sin(yaw))
            y += v / yaw_rate * (-np.cos(yaw + yaw_rate * dt) + np.cos(yaw))
        z += vz * dt
        yaw += yaw_rate * dt
        self.x = np.array([x, y, z, v, yaw, yaw_rate, vz])
        self.P = self.P + self.Q

class IMMKF:
    def __init__(self, models, transition_matrix):
        self.models = models
        self.M = len(models)
        self.transition_matrix = transition_matrix
        self.mu = np.ones(self.M) / self.M

    def _pad_state(self, x, target_dim):
        if len(x) == target_dim:
            return x.copy()
        padded = np.zeros(target_dim)
        copy_dim = min(len(x), target_dim)
        padded[:copy_dim] = x[:copy_dim]
        return padded

    def _pad_cov(self, P, target_dim):
        if P.shape[0] == target_dim:
            return P.copy()
        padded = np.eye(target_dim) * 1000.0
        copy_dim = min(P.shape[0], target_dim)
        padded[:copy_dim, :copy_dim] = P[:copy_dim, :copy_dim]
        return padded

    def interaction(self):
        c_j = np.zeros(self.M)
        for j in range(self.M):
            for i in range(self.M):
                c_j[j] += self.transition_matrix[i, j] * self.mu[i]
        mixing_prob = np.zeros((self.M, self.M))
        for i in range(self.M):
            for j in range(self.M):
                if c_j[j] > 1e-12:
                    mixing_prob[i, j] = self.transition_matrix[i, j] * self.mu[i] / c_j[j]
        for j in range(self.M):
            target_dim = len(self.models[j].x)
            mixed_x = np.zeros(target_dim)
            mixed_P = np.zeros((target_dim, target_dim))
            for i in range(self.M):
                xi = self._pad_state(self.models[i].x, target_dim)
                mixed_x += mixing_prob[i, j] * xi
            for i in range(self.M):
                xi = self._pad_state(self.models[i].x, target_dim)
                Pi = self._pad_cov(self.models[i].P, target_dim)
                dx = (xi - mixed_x).reshape(-1, 1)
                mixed_P += mixing_prob[i, j] * (Pi + dx @ dx.T)
            self.models[j].x = mixed_x
            self.models[j].P = mixed_P
        self.mu = c_j / np.sum(c_j)

    def predict(self):
        self.interaction()
        for m in self.models:
            m.predict()

    def update(self, z):
        likelihoods = np.array([m.update(z) for m in self.models])
        self.mu *= likelihoods
        s = np.sum(self.mu)
        self.mu = self.mu / s if s > 1e-12 else np.ones(self.M) / self.M

    def fused_position(self):
        pos = np.zeros(3)
        for i, m in enumerate(self.models):
            pos += self.mu[i] * m.x[:3]
        return pos

def generate_uav_trajectory():
    gt = []
    dt = 0.1
    x = y = z = 0.0
    vx, vy, vz = 1.0, 0.0, 0.1
    for _ in range(200):
        x += vx*dt; y += vy*dt; z += vz*dt; gt.append([x,y,z])
    ax = 0.05
    for _ in range(200):
        vx += ax*dt; x += vx*dt; y += vy*dt; z += vz*dt; gt.append([x,y,z])
    yaw = 0.0; speed = 2.0; yaw_rate = np.deg2rad(20)
    for _ in range(100):
        yaw += yaw_rate*dt
        x += speed*np.cos(yaw)*dt
        y += speed*np.sin(yaw)*dt
        z += vz*dt
        gt.append([x,y,z])
    return np.array(gt)

def main():
    dt = 0.1
    gt = generate_uav_trajectory()
    measurements = gt + np.random.randn(*gt.shape) * 0.3

    cv = CVModel(dt)
    ca = CAModel(dt)
    singer = SingerModel(dt)
    ct = CTModel(dt)
    models = [cv, ca, singer, ct]
    for m in models:
        m.x[:3] = measurements[0]
    ct.x[3:] = [1.0, 0.0, np.deg2rad(10), 0.1]

    trans = np.array([
        [0.90, 0.03, 0.03, 0.04],
        [0.03, 0.90, 0.03, 0.04],
        [0.03, 0.03, 0.90, 0.04],
        [0.03, 0.03, 0.04, 0.90]
    ])
    imm = IMMKF(models, trans)

    fused_positions = []
    model_probs = []
    cv_errors = []; ca_errors = []; singer_errors = []; ct_errors = []; imm_errors = []

    for idx, z in enumerate(measurements):
        imm.predict()
        imm.update(z)
        fused = imm.fused_position()
        fused_positions.append(fused)
        model_probs.append(imm.mu.copy())
        gt_pos = gt[idx]
        cv_errors.append(np.linalg.norm(cv.x[:3] - gt_pos))
        ca_errors.append(np.linalg.norm(ca.x[:3] - gt_pos))
        singer_errors.append(np.linalg.norm(singer.x[:3] - gt_pos))
        ct_errors.append(np.linalg.norm(ct.x[:3] - gt_pos))
        imm_errors.append(np.linalg.norm(fused - gt_pos))

    fused_positions = np.array(fused_positions)
    model_probs = np.array(model_probs)

    # RMSE statistics
    rmse = {
        'CV': float(np.sqrt(np.mean(np.square(cv_errors)))),
        'CA': float(np.sqrt(np.mean(np.square(ca_errors)))),
        'Singer': float(np.sqrt(np.mean(np.square(singer_errors)))),
        'CT': float(np.sqrt(np.mean(np.square(ct_errors)))),
        'IMMKF': float(np.sqrt(np.mean(np.square(imm_errors)))),
    }

    rmse_path = os.path.join(RESULT_DIR, 'rmse_results.txt')
    with open(rmse_path, 'w') as f:
        for k, v in rmse.items():
            f.write(f'{k}: {v:.4f} m\n')

    plt.figure(figsize=(8,5))
    plt.bar(list(rmse.keys()), list(rmse.values()))
    plt.ylabel('RMSE (m)')
    plt.title('RMSE Comparison')
    plt.grid(True, axis='y', alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(RESULT_DIR, 'rmse_bar.png'), dpi=200)
    plt.close()

    print('\nRMSE Results:')
    for k, v in rmse.items():
        print(f'{k}: {v:.4f} m')

    fig = plt.figure(figsize=(20, 7))
    ax3d = fig.add_subplot(131, projection='3d')
    ax_prob = fig.add_subplot(132)
    ax_err = fig.add_subplot(133)

    def update(frame):
        ax3d.clear(); ax_prob.clear(); ax_err.clear()
        ax3d.plot(gt[:frame,0], gt[:frame,1], gt[:frame,2], label='Ground Truth')
        ax3d.scatter(measurements[:frame,0], measurements[:frame,1], measurements[:frame,2], s=5, alpha=0.4, label='Measurements')
        ax3d.plot(fused_positions[:frame,0], fused_positions[:frame,1], fused_positions[:frame,2], label='IMMKF')
        ax3d.set_title('3D UAV Trajectory'); ax3d.legend()

        t = np.arange(frame)
        labels = ['CV','CA','Singer','CT']
        for i in range(4):
            ax_prob.plot(t, model_probs[:frame, i], label=labels[i])
        ax_prob.set_ylim([0,1]); ax_prob.set_title('Model Probabilities'); ax_prob.legend(); ax_prob.grid(True)

        ax_err.plot(t, cv_errors[:frame], label='CV')
        ax_err.plot(t, ca_errors[:frame], label='CA')
        ax_err.plot(t, singer_errors[:frame], label='Singer')
        ax_err.plot(t, ct_errors[:frame], label='CT')
        ax_err.plot(t, imm_errors[:frame], '--', linewidth=3, label='IMMKF')
        ax_err.set_title('Prediction Error'); ax_err.legend(); ax_err.grid(True)

    anim = FuncAnimation(fig, update, frames=len(gt), interval=50)
    gif_path = os.path.join(RESULT_DIR, 'immkf_demo.gif')
    png_path = os.path.join(RESULT_DIR, 'immkf_final.png')
    anim.save(gif_path, writer=PillowWriter(fps=20))
    plt.savefig(png_path, dpi=200)
    plt.show()

if __name__ == '__main__':
    main()
