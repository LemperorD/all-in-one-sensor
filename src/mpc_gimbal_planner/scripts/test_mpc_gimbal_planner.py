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

    angle_yaw = 0.0
    rate_yaw = 0.0
    angle_pitch = 0.0
    rate_pitch = 0.0
    last_target_yaw = None
    last_target_pitch = None
    last_future_yaw = None
    last_future_pitch = None

    # PID control parameters
    Kp = 8.0
    Ki = 1.0
    Kd = 0.0
    integral_error_yaw = 0.0
    integral_error_pitch = 0.0
    prev_error_yaw = 0.0
    prev_error_pitch = 0.0

    times=[]; target_yaw_angles=[]; pred_target_yaw_angles=[]; mpc_target_yaw_angles=[]; gimbal_yaw_angles=[]; yaw_errors=[]; yaw_rates=[]
    target_pitch_angles=[]; pred_target_pitch_angles=[]; mpc_target_pitch_angles=[]; gimbal_pitch_angles=[]; pitch_errors=[]; pitch_rates=[]
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
        future_pitch_raw = [np.arctan2(p[2], np.hypot(p[0], p[1])) for p in pred]
        future_ref = []
        future_pitch = []
        previous_future_yaw = last_future_yaw
        previous_future_pitch = last_future_pitch
        for value in future_ref_raw:
            unwrapped_value = unwrap_angle(value, previous_future_yaw)
            future_ref.append(unwrapped_value)
            previous_future_yaw = unwrapped_value
        for value in future_pitch_raw:
            unwrapped_value = unwrap_angle(value, previous_future_pitch)
            future_pitch.append(unwrapped_value)
            previous_future_pitch = unwrapped_value
        future_ref = np.array(future_ref)
        future_pitch = np.array(future_pitch)
        last_future_yaw = future_ref[0]
        last_future_pitch = future_pitch[0]
        future_ref_history.append(np.rad2deg(future_ref))
        seq_yaw = mpc.solve_axis(angle_yaw, future_ref)
        seq_pitch = mpc.solve_axis(angle_pitch, future_pitch)
        target_yaw = seq_yaw[0]
        target_pitch = seq_pitch[0]

        # PID control for yaw
        error_yaw = target_yaw - angle_yaw
        integral_error_yaw += error_yaw * dt
        d_error_yaw = (error_yaw - prev_error_yaw) / dt if dt > 0 else 0
        pid_output_yaw = Kp * error_yaw + Ki * integral_error_yaw + Kd * d_error_yaw
        rate_yaw = pid_output_yaw
        rate_yaw = np.clip(rate_yaw, -mpc.max_rate, mpc.max_rate)
        angle_yaw += rate_yaw * dt
        prev_error_yaw = error_yaw

        # PID control for pitch
        error_pitch = target_pitch - angle_pitch
        integral_error_pitch += error_pitch * dt
        d_error_pitch = (error_pitch - prev_error_pitch) / dt if dt > 0 else 0
        pid_output_pitch = Kp * error_pitch + Ki * integral_error_pitch + Kd * d_error_pitch
        rate_pitch = pid_output_pitch
        rate_pitch = np.clip(rate_pitch, -mpc.max_rate, mpc.max_rate)
        angle_pitch += rate_pitch * dt
        prev_error_pitch = error_pitch

        true_yaw = np.arctan2(gt[1], gt[0])
        true_yaw = unwrap_angle(true_yaw, last_target_yaw)
        last_target_yaw = true_yaw

        true_pitch = np.arctan2(gt[2], np.hypot(gt[0], gt[1]))
        true_pitch = unwrap_angle(true_pitch, last_target_pitch)
        last_target_pitch = true_pitch

        times.append(t)
        target_yaw_angles.append(np.rad2deg(true_yaw))
        pred_target_yaw_angles.append(np.rad2deg(future_ref[0]))
        mpc_target_yaw_angles.append(np.rad2deg(target_yaw))
        gimbal_yaw_angles.append(np.rad2deg(angle_yaw))
        yaw_errors.append(np.rad2deg(true_yaw - angle_yaw))
        yaw_rates.append(np.rad2deg(rate_yaw))

        target_pitch_angles.append(np.rad2deg(true_pitch))
        pred_target_pitch_angles.append(np.rad2deg(future_pitch[0]))
        mpc_target_pitch_angles.append(np.rad2deg(target_pitch))
        gimbal_pitch_angles.append(np.rad2deg(angle_pitch))
        pitch_errors.append(np.rad2deg(true_pitch - angle_pitch))
        pitch_rates.append(np.rad2deg(rate_pitch))

        gt_history.append(gt)
        pred_history.append(pred)

    pd.DataFrame({
        'time': times,
        'target_yaw_deg': target_yaw_angles,
        'pred_target_yaw_deg': pred_target_yaw_angles,
        'mpc_target_yaw_deg': mpc_target_yaw_angles,
        'gimbal_yaw_deg': gimbal_yaw_angles,
        'yaw_error_deg': yaw_errors,
        'yaw_rate_deg_s': yaw_rates,
        'target_pitch_deg': target_pitch_angles,
        'pred_target_pitch_deg': pred_target_pitch_angles,
        'mpc_target_pitch_deg': mpc_target_pitch_angles,
        'gimbal_pitch_deg': gimbal_pitch_angles,
        'pitch_error_deg': pitch_errors,
        'pitch_rate_deg_s': pitch_rates
    }).to_csv(os.path.join(RESULT_DIR, 'imm_mpc_data.csv'), index=False)

    fig = plt.figure(figsize=(20, 10))
    gs = fig.add_gridspec(2, 3)

    margin = 5.0
    ang_min = min(min(target_yaw_angles), min(gimbal_yaw_angles)) - margin
    ang_max = max(max(target_yaw_angles), max(gimbal_yaw_angles)) + margin
    pitch_min = min(min(target_pitch_angles), min(gimbal_pitch_angles)) - margin
    pitch_max = max(max(target_pitch_angles), max(gimbal_pitch_angles)) + margin
    yaw_err_min = min(yaw_errors) - margin
    yaw_err_max = max(yaw_errors) + margin
    pitch_err_min = min(pitch_errors) - margin
    pitch_err_max = max(pitch_errors) + margin
    yaw_rate_min = min(yaw_rates) - margin
    yaw_rate_max = max(yaw_rates) + margin
    pitch_rate_min = min(pitch_rates) - margin
    pitch_rate_max = max(pitch_rates) + margin
    ax3d = fig.add_subplot(gs[0, 0], projection='3d')
    ax_track = fig.add_subplot(gs[0, 1])
    ax_pitch = fig.add_subplot(gs[0, 2])
    ax_err = fig.add_subplot(gs[1, 0])
    ax_rate = fig.add_subplot(gs[1, 1])
    ax_future = fig.add_subplot(gs[1, 2])

    line_gt_3d, = ax3d.plot([], [], [], label='Target')
    line_pred_3d, = ax3d.plot([], [], [], '--', label='Prediction')
    point_target_3d, = ax3d.plot([], [], [], 'o')
    ax3d.set_xlim(-25, 25); ax3d.set_ylim(-25, 25); ax3d.set_zlim(0, 10)
    ax3d.legend()
    ax3d.set_title('3D Target + Gimbal')

    line_target, = ax_track.plot([], [], label='Real yaw')
    line_pred_target, = ax_track.plot([], [], label='Pred yaw')
    line_mpc_target, = ax_track.plot([], [], label='MPC yaw')
    line_gimbal, = ax_track.plot([], [], label='Gimbal yaw')
    ax_track.set_xlim(0, sim_time); ax_track.set_ylim(ang_min, ang_max)
    ax_track.set_title('Yaw: Real / Predicted / MPC / Gimbal')
    ax_track.set_xlabel('Time (s)')
    ax_track.set_ylabel('Angle (deg)')
    ax_track.grid(True); ax_track.legend()

    line_pitch_target, = ax_pitch.plot([], [], label='Real pitch')
    line_pitch_pred_target, = ax_pitch.plot([], [], label='Pred pitch')
    line_pitch_mpc_target, = ax_pitch.plot([], [], label='MPC pitch')
    line_pitch_gimbal, = ax_pitch.plot([], [], label='Gimbal pitch')
    ax_pitch.set_xlim(0, sim_time); ax_pitch.set_ylim(pitch_min, pitch_max)
    ax_pitch.set_title('Pitch: Real / Predicted / MPC / Gimbal')
    ax_pitch.set_xlabel('Time (s)')
    ax_pitch.set_ylabel('Angle (deg)')
    ax_pitch.grid(True); ax_pitch.legend()

    line_error, = ax_err.plot([], [], label='Yaw error')
    ax_err.set_xlim(0, sim_time); ax_err.set_ylim(yaw_err_min, yaw_err_max)
    ax_err.set_title('Yaw Tracking Error')
    ax_err.set_xlabel('Time (s)')
    ax_err.set_ylabel('Error (deg)')
    ax_err.grid(True)

    line_rate, = ax_rate.plot([], [], label='Yaw rate')
    ax_rate.set_xlim(0, sim_time); ax_rate.set_ylim(yaw_rate_min, yaw_rate_max)
    ax_rate.set_title('Yaw Angular Rate')
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
        line_target.set_data(t, target_yaw_angles[:frame])
        line_pred_target.set_data(t, pred_target_yaw_angles[:frame])
        line_mpc_target.set_data(t, mpc_target_yaw_angles[:frame])
        line_gimbal.set_data(t, gimbal_yaw_angles[:frame])
        line_pitch_target.set_data(t, target_pitch_angles[:frame])
        line_pitch_pred_target.set_data(t, pred_target_pitch_angles[:frame])
        line_pitch_mpc_target.set_data(t, mpc_target_pitch_angles[:frame])
        line_pitch_gimbal.set_data(t, gimbal_pitch_angles[:frame])
        line_error.set_data(t, yaw_errors[:frame])
        line_rate.set_data(t, yaw_rates[:frame])

        err_window = np.array(yaw_errors[:frame])
        err_span = max(np.max(err_window) - np.min(err_window), 1.0)
        err_margin = max(2.0, 0.2 * err_span)
        ax_err.set_ylim(np.min(err_window) - err_margin, np.max(err_window) + err_margin)

        pitch_err_window = np.array(pitch_errors[:frame])
        pitch_err_span = max(np.max(pitch_err_window) - np.min(pitch_err_window), 1.0)
        pitch_err_margin = max(2.0, 0.2 * pitch_err_span)
        ax_pitch.set_ylim(np.min(pitch_err_window) - pitch_err_margin, np.max(pitch_err_window) + pitch_err_margin)

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
        future_window = np.concatenate(future_ref_history[:frame])
        future_span = max(np.max(future_window) - np.min(future_window), 1.0)
        future_margin = max(5.0, 0.15 * future_span)
        ax_future.set_ylim(np.min(future_window) - future_margin, np.max(future_window) + future_margin)

        ax3d.collections.clear()
        yaw = np.deg2rad(gimbal_yaw_angles[frame-1])
        pitch = np.deg2rad(gimbal_pitch_angles[frame-1])
        ax3d.quiver(0, 0, 0,
                15*np.cos(pitch)*np.cos(yaw),
                15*np.cos(pitch)*np.sin(yaw),
                15*np.sin(pitch),
                length=1.0)

        return line_target, line_pred_target, line_mpc_target, line_gimbal, line_pitch_target, line_pitch_pred_target, line_pitch_mpc_target, line_pitch_gimbal, line_error, line_rate, line_future, line_gt_3d, line_pred_3d, point_target_3d

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
