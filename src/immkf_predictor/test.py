#!/usr/bin/env python3
"""
IMM-KF comparison (STATIC PNG OUTPUT)
Trajectory:

s = 0.5 * (1 - cos(pi t / T))
x = 10 s cos(0.6 t)
y = 10 s sin(0.6 t)

Outputs:
- /tmp/imm_with_ct.png
- /tmp/imm_no_ct.png
- /tmp/imm_error_compare.png
"""

import numpy as np
import matplotlib.pyplot as plt

# ===================== Trajectory =====================
def generate_ground_truth(T=40.0, dt=0.1):
    t = np.arange(0, T, dt)
    pos = []

    for ti in t:
        s = 0.5 * (1 - np.cos(np.pi * ti / T))
        x = 10 * s * np.cos(0.6 * ti)
        y = 10 * s * np.sin(0.6 * ti)
        pos.append([x, y])

    return t, np.array(pos)


def add_noise(traj, std=0.15):
    return traj + np.random.randn(*traj.shape) * std


# ===================== KF =====================
class KF:
    def __init__(self, dim_x):
        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x)
        self.F = np.eye(dim_x)
        self.Q = np.eye(dim_x) * 0.01
        self.H = None
        self.R = np.eye(2) * 0.2

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
    kf = KF(4)
    F = np.eye(4)
    F[0, 2] = dt
    F[1, 3] = dt
    kf.F = F
    kf.H = np.hstack([np.eye(2), np.zeros((2,2))])
    return kf


def create_ca(dt):
    kf = KF(6)
    F = np.eye(6)
    for i in range(2):
        F[i, i+2] = dt
        F[i, i+4] = 0.5 * dt**2
        F[i+2, i+4] = dt
    kf.F = F
    kf.H = np.hstack([np.eye(2), np.zeros((2,4))])
    return kf


def create_singer(dt, alpha=0.8):
    kf = KF(6)
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
    kf.H = np.hstack([np.eye(2), np.zeros((2,4))])
    return kf


def create_ct(dt, omega=0.6):
    kf = KF(5)
    F = np.eye(5)

    if abs(omega) > 1e-4:
        s = np.sin(omega * dt)
        c = np.cos(omega * dt)
        F[0,2] = s/omega
        F[0,3] = -(1-c)/omega
        F[1,2] = (1-c)/omega
        F[1,3] = s/omega
        F[2,2] = c
        F[2,3] = -s
        F[3,2] = s
        F[3,3] = c

    kf.F = F
    kf.H = np.hstack([np.eye(2), np.zeros((2,3))])
    return kf


# ===================== IMM =====================
class IMM:
    def __init__(self, models):
        self.models = models
        self.mu = np.ones(len(models)) / len(models)

    def step(self, z):
        likelihoods = []

        for m in self.models:
            m.predict()
            m.update(z)
            innov = z - m.H @ m.x
            likelihoods.append(np.exp(-0.5*np.linalg.norm(innov)))

        likelihoods = np.array(likelihoods)
        self.mu = self.mu * likelihoods
        self.mu /= np.sum(self.mu)

        fused = np.zeros(2)
        for i,m in enumerate(self.models):
            fused += self.mu[i]*m.x[:2]

        return fused


# ===================== Run =====================
def run(models):
    dt = 0.1
    t, gt = generate_ground_truth()
    meas = add_noise(gt)

    imm = IMM(models)

    preds = []
    for z in meas:
        preds.append(imm.step(z))

    preds = np.array(preds)

    err = np.linalg.norm(preds - gt, axis=1)

    return t, gt, meas, preds, err


# ===================== Main =====================
if __name__ == '__main__':
    dt = 0.1

    # WITH CT
    models_ct = [create_cv(dt), create_ca(dt), create_singer(dt), create_ct(dt)]
    t, gt, meas, pred_ct, err_ct = run(models_ct)

    # WITHOUT CT
    models_no_ct = [create_cv(dt), create_ca(dt), create_singer(dt)]
    _, _, _, pred_no_ct, err_no_ct = run(models_no_ct)

    # ===== Trajectory Comparison =====
    plt.figure(figsize=(8,8))
    plt.plot(gt[:,0], gt[:,1], label='Ground Truth')
    plt.scatter(meas[:,0], meas[:,1], s=5, alpha=0.5)
    plt.plot(pred_ct[:,0], pred_ct[:,1], label='IMM + CT', linewidth=2.5)
    plt.plot(pred_no_ct[:,0], pred_no_ct[:,1], '--', label='IMM no CT')
    plt.legend(); plt.grid()
    plt.title('Trajectory Comparison')
    plt.savefig('/tmp/imm_compare.png', dpi=300)

    # ===== Error Comparison =====
    plt.figure(figsize=(10,5))
    plt.plot(t, err_ct, label='IMM + CT', linewidth=2.5)
    plt.plot(t, err_no_ct, '--', label='IMM no CT')
    plt.legend(); plt.grid()
    plt.title('Error Comparison')
    plt.xlabel('time'); plt.ylabel('error')
    plt.savefig('/tmp/imm_error_compare.png', dpi=300)

    print('Saved:')
    print('/tmp/imm_compare.png')
    print('/tmp/imm_error_compare.png')
