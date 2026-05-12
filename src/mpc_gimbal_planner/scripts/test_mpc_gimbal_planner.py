#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import copy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

# ============================
# 输出目录
# ============================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULT_DIR = os.path.join(SCRIPT_DIR, "results")
os.makedirs(RESULT_DIR, exist_ok=True)

# ============================
# 基础 KF
# ============================
class KalmanFilter:
    def __init__(self, dim_x, dim_z):
        self.x = np.zeros(dim_x)
        self.P = np.eye(dim_x)
        self.F = np.eye(dim_x)
        self.H = np.zeros((dim_z, dim_x))
        self.Q = np.eye(dim_x) * 0.05
        self.R = np.eye(dim_z) * 0.3

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(len(self.x)) - K @ self.H) @ self.P
        det_s = max(np.linalg.det(S), 1e-6)
        ll = np.exp(-0.5 * y.T @ np.linalg.inv(S) @ y) / np.sqrt(((2*np.pi)**3) * det_s)
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
        yaw += yaw_rate * self.dt
        pitch += pitch_rate * self.dt
        vx = v * np.cos(pitch) * np.cos(yaw)
        vy = v * np.cos(pitch) * np.sin(yaw)
        vz = v * np.sin(pitch)
        x += vx * self.dt
        y += vy * self.dt
        z += vz * self.dt
        self.x = np.array([x, y, z, v, yaw, pitch, yaw_rate, pitch_rate])
        self.P = self.P + self.Q

class IMMKF:
    def __init__(self, models, trans):
        self.models = models
        self.trans = trans
        self.mu = np.ones(len(models)) / len(models)

    def predict(self):
        for m in self.models:
            m.predict()

    def update(self, z):
        ll = np.array([m.update(z) for m in self.models])
        self.mu *= ll
        s = np.sum(self.mu)
        self.mu = self.mu / s if s > 1e-12 else np.ones(len(self.models)) / len(self.models)

    def fused_position(self):
        pos = np.zeros(3)
        for i, m in enumerate(self.models):
            pos += self.mu[i] * m.x[:3]
        return pos

    def future_positions(self, steps):
        clone = copy.deepcopy(self)
        out = []
        for _ in range(steps):
            clone.predict()
            out.append(clone.fused_position())
        return np.array(out)

# ============================
# MPC 云台控制
# ============================
class MPCGimbal:
    def __init__(self):
        self.horizon_steps = 20
        self.prediction_dt = 0.02
        self.track_weight = 20.0
        self.max_rate = np.deg2rad(250)
        self.angle_min = np.deg2rad(-180)
        self.angle_max = np.deg2rad(180)

    def solve_axis(self, current_angle, ref_seq):
        ref = np.array(ref_seq)
        ref = ref[:self.horizon_steps]
        if len(ref) < self.horizon_steps:
            ref = np.pad(ref, (0, self.horizon_steps - len(ref)), mode='edge')
        max_delta = self.max_rate * self.prediction_dt
        ref[0] = np.clip(ref[0], current_angle - max_delta, current_angle + max_delta)
        return ref

# ============================
# 主程序
# ============================
def main():
    mpc = MPCGimbal()
    dt = mpc.prediction_dt
    sim_time = 12.0
    steps = int(sim_time / dt)

    models = [CVModel(dt), CAModel(dt), CTModel(dt)]
    for m in models:
        m.x[:3] = [20, 0, 5]
    models[-1].x[3:] = [4.0, 0.0, 0.1, 0.2, 0.05]

    trans = np.array([[0.90,0.05,0.05],[0.05,0.90,0.05],[0.05,0.05,0.90]])
    imm = IMMKF(models, trans)

    angle = 0.0
    rate = 0.0
    last_target_yaw = None
    last_future_yaw = None

    times=[]; target_angles=[]; pred_target_angles=[]; mpc_target_angles=[]; gimbal_angles=[]; errors=[]; rates=[]
    gt_history=[]; pred_history=[]; future_ref_history=[]

    def unwrap_angle(angle_rad, previous_angle_rad):
        if previous_angle_rad is None:
            return angle_rad
        return previous_angle_rad + np.arctan2(np.sin(angle_rad - previous_angle_rad), np.cos(angle_rad - previous_angle_rad))

    for k in range(steps):
        t = k * dt
        gt = np.array([
            20*np.cos(0.5*t),
            20*np.sin(0.5*t),
            5 + 2*np.sin(0.8*t)
        ])
        measurement = gt + np.random.randn(3) * 0.25

        imm.predict()
        imm.update(measurement)
        pred = imm.future_positions(mpc.horizon_steps)

        future_ref_raw = [np.arctan2(p[1], p[0]) for p in pred]
        future_ref = []
        previous_future_yaw = last_future_yaw
        for value in future_ref_raw:
            unwrapped_value = unwrap_angle(value, previous_future_yaw)
            future_ref.append(unwrapped_value)
            previous_future_yaw = unwrapped_value
        future_ref = np.array(future_ref)
        last_future_yaw = future_ref[0]
        future_ref_history.append(np.rad2deg(future_ref))
        seq = mpc.solve_axis(angle, future_ref)
        target_angle = seq[0]

        alpha = 18.0
        rate += alpha * (target_angle - angle) * dt
        rate = np.clip(rate, -mpc.max_rate, mpc.max_rate)
        angle += rate * dt

        true_yaw = np.arctan2(gt[1], gt[0])
        true_yaw = unwrap_angle(true_yaw, last_target_yaw)
        last_target_yaw = true_yaw

        times.append(t)
        target_angles.append(np.rad2deg(true_yaw))
        pred_target_angles.append(np.rad2deg(future_ref[0]))
        mpc_target_angles.append(np.rad2deg(target_angle))
        gimbal_angles.append(np.rad2deg(angle))
        errors.append(np.rad2deg(true_yaw - angle))
        rates.append(np.rad2deg(rate))
        gt_history.append(gt)
        pred_history.append(pred)

    pd.DataFrame({
        'time': times,
        'target_deg': target_angles,
        'pred_target_deg': pred_target_angles,
        'mpc_target_deg': mpc_target_angles,
        'gimbal_deg': gimbal_angles,
        'error_deg': errors,
        'rate_deg_s': rates
    }).to_csv(os.path.join(RESULT_DIR, 'imm_mpc_data.csv'), index=False)

    fig = plt.figure(figsize=(20, 10))
    gs = fig.add_gridspec(2, 3)

    margin = 5.0
    ang_min = min(min(target_angles), min(gimbal_angles)) - margin
    ang_max = max(max(target_angles), max(gimbal_angles)) + margin
    err_min = min(errors) - margin
    err_max = max(errors) + margin
    rate_min = min(rates) - margin
    rate_max = max(rates) + margin
    ax3d = fig.add_subplot(gs[0, 0], projection='3d')
    ax_track = fig.add_subplot(gs[0, 1])
    ax_err = fig.add_subplot(gs[0, 2])
    ax_rate = fig.add_subplot(gs[1, 0])
    ax_future = fig.add_subplot(gs[1, 1])

    line_gt_3d, = ax3d.plot([], [], [], label='Target')
    line_pred_3d, = ax3d.plot([], [], [], '--', label='Prediction')
    point_target_3d, = ax3d.plot([], [], [], 'o')
    ax3d.set_xlim(-25, 25); ax3d.set_ylim(-25, 25); ax3d.set_zlim(0, 10)
    ax3d.legend()
    ax3d.set_title('3D Target + Gimbal')

    line_target, = ax_track.plot([], [], label='Real target')
    line_pred_target, = ax_track.plot([], [], label='Predicted target')
    line_mpc_target, = ax_track.plot([], [], label='MPC command')
    line_gimbal, = ax_track.plot([], [], label='Gimbal')
    ax_track.set_xlim(0, sim_time); ax_track.set_ylim(ang_min, ang_max)
    ax_track.set_title('Real / Predicted / MPC / Gimbal')
    ax_track.set_xlabel('Time (s)')
    ax_track.set_ylabel('Angle (deg)')
    ax_track.grid(True); ax_track.legend()

    line_error, = ax_err.plot([], [])
    ax_err.set_xlim(0, sim_time); ax_err.set_ylim(err_min, err_max)
    ax_err.set_title('Tracking Error')
    ax_err.set_xlabel('Time (s)')
    ax_err.set_ylabel('Error (deg)')
    ax_err.grid(True)

    line_rate, = ax_rate.plot([], [])
    ax_rate.set_xlim(0, sim_time); ax_rate.set_ylim(rate_min, rate_max)
    ax_rate.set_title('Gimbal Angular Rate')
    ax_rate.set_xlabel('Time (s)')
    ax_rate.set_ylabel('Rate (deg/s)')
    ax_rate.grid(True)

    line_future, = ax_future.plot([], [], label='IMM future yaw')
    ax_future.set_xlim(0, mpc.horizon_steps - 1)
    ax_future.set_title('IMM Predicted Angle Sequence')
    ax_future.set_xlabel('Prediction Step')
    ax_future.set_ylabel('Angle (deg)')
    ax_future.grid(True)
    ax_future.legend()

    def update(frame):
        if frame < 2:
            frame = 2
        t = times[:frame]
        line_target.set_data(t, target_angles[:frame])
        line_pred_target.set_data(t, pred_target_angles[:frame])
        line_mpc_target.set_data(t, mpc_target_angles[:frame])
        line_gimbal.set_data(t, gimbal_angles[:frame])
        line_error.set_data(t, errors[:frame])
        line_rate.set_data(t, rates[:frame])

        gt = np.array(gt_history[:frame])
        line_gt_3d.set_data(gt[:,0], gt[:,1])
        line_gt_3d.set_3d_properties(gt[:,2])
        point_target_3d.set_data([gt[-1,0]], [gt[-1,1]])
        point_target_3d.set_3d_properties([gt[-1,2]])

        pred = pred_history[frame-1]
        line_pred_3d.set_data(pred[:,0], pred[:,1])
        line_pred_3d.set_3d_properties(pred[:,2])

        future_ref = future_ref_history[frame-1]
        future_steps = np.arange(len(future_ref))
        line_future.set_data(future_steps, future_ref)
        ax_future.set_ylim(np.min(future_ref) - 5, np.max(future_ref) + 5)

        ax3d.collections.clear()
        yaw = np.deg2rad(gimbal_angles[frame-1])
        ax3d.quiver(0, 0, 0, 15*np.cos(yaw), 15*np.sin(yaw), 0, length=1.0)

        return line_target, line_pred_target, line_mpc_target, line_gimbal, line_error, line_rate, line_future, line_gt_3d, line_pred_3d, point_target_3d

    ani = FuncAnimation(fig, update, frames=len(times), interval=20)
    gif_path = os.path.join(RESULT_DIR, 'imm_mpc_tracking.gif')
    png_path = os.path.join(RESULT_DIR, 'final.png')
    ani.save(gif_path, writer=PillowWriter(fps=30))
    plt.savefig(png_path, dpi=180)
    plt.close(fig)
    print(f'Saved GIF to {gif_path}')
    print(f'Saved figure to {png_path}')

if __name__ == '__main__':
    main()
