#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULT_DIR = os.path.join(SCRIPT_DIR, 'results')
os.makedirs(RESULT_DIR, exist_ok=True)

# ================= Base KF =================
class KalmanFilter:
    def __init__(self, dim_x, dim_z):
        self.dim_x = dim_x
        self.dim_z = dim_z
        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x)
        self.F = np.eye(dim_x)
        self.H = np.zeros((dim_z, dim_x))
        self.Q = np.eye(dim_x) * 0.05
        self.R = np.eye(dim_z) * 0.5

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(self.dim_x) - K @ self.H) @ self.P
        det_s = max(np.linalg.det(S), 1e-6)
        ll = np.exp(-0.5 * y.T @ np.linalg.inv(S) @ y) / np.sqrt(((2*np.pi)**self.dim_z) * det_s)
        return max(ll, 1e-12)

class CVModel(KalmanFilter):
    def __init__(self, dt):
        super().__init__(6, 3)
        for i in range(3):
            self.F[i, i+3] = dt
            self.H[i, i] = 1

class CAModel(KalmanFilter):
    def __init__(self, dt):
        super().__init__(9, 3)
        for i in range(3):
            self.F[i, i+3] = dt
            self.F[i, i+6] = 0.5 * dt * dt
            self.F[i+3, i+6] = dt
            self.H[i, i] = 1

class SingerModel(KalmanFilter):
    def __init__(self, dt, alpha=0.95):
        super().__init__(9, 3)
        e = np.exp(-alpha * dt)
        for i in range(3):
            self.F[i, i+3] = dt
            self.F[i+3, i+6] = (1 - e)
            self.F[i+6, i+6] = e
            self.H[i, i] = 1

class CTModel(KalmanFilter):
    def __init__(self, dt):
        super().__init__(8, 3)
        self.dt = dt
        for i in range(3):
            self.H[i, i] = 1

    def predict(self):
        x, y, z, v, yaw, pitch, yaw_rate, pitch_rate = self.x
        dt = self.dt

        # 更新角度
        yaw += yaw_rate * dt
        pitch += pitch_rate * dt

        # 三维速度分解
        vx = v * np.cos(pitch) * np.cos(yaw)
        vy = v * np.cos(pitch) * np.sin(yaw)
        vz = v * np.sin(pitch)

        # 更新位置
        x += vx * dt
        y += vy * dt
        z += vz * dt

        self.x = np.array([
            x, y, z,
            v,
            yaw,
            pitch,
            yaw_rate,
            pitch_rate
        ])

        self.P = self.P + self.Q

# ================= IMM =================
class IMMKF:
    def __init__(self, models, trans):
        self.models = models
        self.trans = trans
        self.M = len(models)
        self.mu = np.ones(self.M)/self.M

    def _pad_state(self, x, target):
        out = np.zeros(target)
        c = min(len(x), target)
        out[:c] = x[:c]
        return out

    def _pad_cov(self, P, target):
        out = np.eye(target) * 1000
        c = min(P.shape[0], target)
        out[:c, :c] = P[:c, :c]
        return out

    def interaction(self):
        c = np.zeros(self.M)
        for j in range(self.M):
            for i in range(self.M): c[j] += self.trans[i, j] * self.mu[i]
        mix = np.zeros((self.M, self.M))
        for i in range(self.M):
            for j in range(self.M):
                if c[j] > 1e-12:
                    mix[i, j] = self.trans[i, j] * self.mu[i] / c[j]
        for j in range(self.M):
            td = len(self.models[j].x)
            mx = np.zeros(td)
            for i in range(self.M):
                mx += mix[i, j] * self._pad_state(self.models[i].x, td)
            mp = np.zeros((td, td))
            for i in range(self.M):
                xi = self._pad_state(self.models[i].x, td)
                Pi = self._pad_cov(self.models[i].P, td)
                dx = (xi - mx).reshape(-1, 1)
                mp += mix[i, j] * (Pi + dx @ dx.T)
            self.models[j].x = mx; self.models[j].P = mp
        self.mu = c / np.sum(c)

    def predict(self):
        self.interaction()
        for m in self.models: m.predict()

    def update(self, z):
        ll = np.array([m.update(z) for m in self.models])
        self.mu = self.mu * ll
        s = np.sum(self.mu)
        self.mu = self.mu / s if s > 1e-12 else np.ones(self.M)/self.M

    def fused_position(self):
        pos = np.zeros(3)
        for i, m in enumerate(self.models): pos += self.mu[i] * m.x[:3]
        return pos

def generate_trajectory(mode, dt=0.1):
    t = np.arange(0, 20, dt)
    gt = []
    if mode == 'figure8':
        A, w = 10, 0.4
        for ti in t: gt.append([A*np.sin(w*ti), A*np.sin(w*ti)*np.cos(w*ti), 0.2*ti])
    elif mode == 'spiral':
        R, w = 8, 0.5
        for ti in t: gt.append([R*np.cos(w*ti), R*np.sin(w*ti), 0.3*ti])
    elif mode == 'circle_acc':
        base_r, w = 10, 0.5
        for ti in t:
            r = base_r + 2*np.sin(0.3*ti)
            gt.append([r*np.cos(w*ti), r*np.sin(w*ti), 0.1*ti])
    else:
        x=y=z=0.0; vx=1.0; vy=0.0; vz=0.1
        for _ in range(50): x+=vx*dt; y+=vy*dt; z+=vz*dt; gt.append([x,y,z])
        ax=0.05
        for _ in range(50): vx+=ax*dt; x+=vx*dt; z+=vz*dt; gt.append([x,y,z])
        yaw=0; speed=2.0; yaw_rate=np.deg2rad(20)
        for _ in range(100): yaw+=yaw_rate*dt; x+=speed*np.cos(yaw)*dt; y+=speed*np.sin(yaw)*dt; z+=vz*dt; gt.append([x,y,z])
    return np.array(gt)

def run_single(mode):
    dt = 0.1
    gt = generate_trajectory(mode, dt)
    measurements = gt + np.random.randn(*gt.shape) * 0.1
    models = [CVModel(dt), CAModel(dt), SingerModel(dt), CTModel(dt)]
    for m in models: m.x[:3] = measurements[0]
    # models[-1].x[3:] = [1.0, 0.0, np.deg2rad(10), 0.1]
    models[-1].x[3:] = [
        2.0,                # speed
        0.0,                # yaw
        0.0,                # pitch
        np.deg2rad(10),     # yaw_rate
        np.deg2rad(3)       # pitch_rate
    ]
    trans = np.array([[0.9,0.03,0.03,0.04],[0.03,0.9,0.03,0.04],[0.03,0.03,0.9,0.04],[0.03,0.03,0.04,0.9]])
    imm = IMMKF(models, trans)
    fused=[]; probs=[]; errs=[]
    per_model_err = [[] for _ in range(4)]
    for idx, z in enumerate(measurements):
        imm.predict(); imm.update(z)
        fp = imm.fused_position(); fused.append(fp); probs.append(imm.mu.copy())
        gtp = gt[idx]
        for i, m in enumerate(models): per_model_err[i].append(np.linalg.norm(m.x[:3]-gtp))
        errs.append(np.linalg.norm(fp - gtp))
    fused = np.array(fused); probs = np.array(probs)
    rmse = [np.sqrt(np.mean(np.square(e))) for e in per_model_err] + [np.sqrt(np.mean(np.square(errs)))]
    with open(os.path.join(RESULT_DIR, f'{mode}_rmse.txt'), 'w') as f:
        for name, val in zip(['CV','CA','Singer','CT','IMMKF'], rmse): f.write(f'{name}: {val:.4f} m\n')
    fig = plt.figure(figsize=(18,6))
    ax1 = fig.add_subplot(131, projection='3d'); ax2 = fig.add_subplot(132); ax3 = fig.add_subplot(133)
    def update(frame):
        ax1.clear(); ax2.clear(); ax3.clear(); t = np.arange(frame)
        ax1.plot(gt[:frame,0], gt[:frame,1], gt[:frame,2]); ax1.scatter(measurements[:frame,0], measurements[:frame,1], measurements[:frame,2], s=4, alpha=0.3); ax1.plot(fused[:frame,0], fused[:frame,1], fused[:frame,2])
        for i, lbl in enumerate(['CV','CA','Singer','CT']): ax2.plot(t, probs[:frame,i], label=lbl)
        for i, lbl in enumerate(['CV','CA','Singer','CT']): ax3.plot(t, per_model_err[i][:frame], label=lbl)
        ax3.plot(t, errs[:frame], '--', linewidth=2, label='IMMKF')
        ax2.set_ylim(0,1); ax2.legend(); ax3.legend(); ax2.grid(True); ax3.grid(True)
        ax1.set_title(mode)
    anim = FuncAnimation(fig, update, frames=len(gt), interval=50)
    anim.save(os.path.join(RESULT_DIR, f'{mode}_demo.gif'), writer=PillowWriter(fps=20))
    plt.savefig(os.path.join(RESULT_DIR, f'{mode}_final.png'), dpi=180)
    plt.close(fig)
    return mode, rmse[-1]

def main():
    modes = ['composite', 'figure8', 'spiral', 'circle_acc']
    summary = []
    for m in modes:
        print('Running', m)
        summary.append(run_single(m))
    with open(os.path.join(RESULT_DIR, 'all_rmse_summary.txt'), 'w') as f:
        for name, val in summary: f.write(f'{name}: {val:.4f} m\n')
    print('Done. Results saved to', RESULT_DIR)

if __name__ == '__main__':
    main()
