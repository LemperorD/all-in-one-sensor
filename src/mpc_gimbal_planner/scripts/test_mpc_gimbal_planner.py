import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


class MPCGimbal:

    def __init__(self):

        self.horizon_steps = 20
        self.prediction_dt = 0.02

        self.track_weight = 20.0
        self.smooth_weight = 10.0
        self.control_weight = 2.0

        self.max_rate = np.deg2rad(250)

        self.angle_min = np.deg2rad(-180)
        self.angle_max = np.deg2rad(180)

    def clamp(self, value, min_value, max_value):
        return np.minimum(np.maximum(value, min_value), max_value)

    def expand_reference(self, reference_sequence):

        if len(reference_sequence) == 0:
            return np.zeros(self.horizon_steps)

        expanded = []

        for i in range(self.horizon_steps):
            idx = min(i, len(reference_sequence) - 1)
            expanded.append(reference_sequence[idx])

        return np.array(expanded)

    def project_sequence(self, sequence, current_angle):

        projected = sequence.copy()

        max_delta = self.max_rate * self.prediction_dt

        for _ in range(4):

            prev = current_angle

            for i in range(len(projected)):

                projected[i] = self.clamp(
                    projected[i],
                    self.angle_min,
                    self.angle_max
                )

                projected[i] = self.clamp(
                    projected[i],
                    prev - max_delta,
                    prev + max_delta
                )

                prev = projected[i]

        return projected

    def add_quadratic_term(self, H, g, coeffs, constant, weight):

        for row, coeff_row in coeffs:
            for col, coeff_col in coeffs:
                H[row, col] += 2.0 * weight * coeff_row * coeff_col

            g[row] += -2.0 * weight * constant * coeff_row

    def solve_axis(self,
                   current_angle,
                   current_rate,
                   reference_sequence):

        N = self.horizon_steps

        ref = self.expand_reference(reference_sequence)

        H = np.zeros((N, N))
        g = np.zeros(N)

        # tracking term
        for i in range(N):

            H[i, i] += 2.0 * self.track_weight
            g[i] += -2.0 * self.track_weight * ref[i]

        # smooth term
        for i in range(N):

            if i == 0:

                H[0, 0] += 2.0 * self.smooth_weight
                g[0] += -2.0 * self.smooth_weight * current_angle

            else:

                H[i, i] += 2.0 * self.smooth_weight
                H[i - 1, i - 1] += 2.0 * self.smooth_weight

                H[i, i - 1] += -2.0 * self.smooth_weight
                H[i - 1, i] += -2.0 * self.smooth_weight

        # control term
        w = self.control_weight / (self.prediction_dt ** 2)

        if N >= 1:
            self.add_quadratic_term(
                H,
                g,
                [(0, 1.0)],
                current_angle + self.prediction_dt * current_rate,
                w
            )

        if N >= 2:
            self.add_quadratic_term(
                H,
                g,
                [(1, 1.0), (0, -2.0)],
                current_angle,
                w
            )

        for i in range(2, N):

            self.add_quadratic_term(
                H,
                g,
                [
                    (i, 1.0),
                    (i - 1, -2.0),
                    (i - 2, 1.0)
                ],
                0.0,
                w
            )

        H += 1e-6 * np.eye(N)

        solution = np.linalg.solve(H, -g)

        solution = self.project_sequence(solution, current_angle)

        return solution


# ==========================================================
# 创建结果目录
# ==========================================================

results_dir = "results"
os.makedirs(results_dir, exist_ok=True)

# ==========================================================
# 创建 MPC
# ==========================================================

mpc = MPCGimbal()

# ==========================================================
# 仿真参数
# ==========================================================

sim_time = 12.0

dt = mpc.prediction_dt

steps = int(sim_time / dt)

# 云台状态
angle = 0.0
rate = 0.0

# 数据记录
times = []
references = []
gimbal_angles = []
errors = []
rates = []

# ==========================================================
# 仿真
# ==========================================================

for k in range(steps):

    t = k * dt

    # 复杂目标运动
    target = np.deg2rad(
        25.0 * np.sin(0.6 * t)
        + 10.0 * np.sin(2.5 * t)
        + 5.0 * np.sin(6.0 * t)
    )

    # 未来参考轨迹
    future_ref = []

    for i in range(mpc.horizon_steps):

        future_t = t + i * dt

        ref = np.deg2rad(
            25.0 * np.sin(0.6 * future_t)
            + 10.0 * np.sin(2.5 * future_t)
            + 5.0 * np.sin(6.0 * future_t)
        )

        future_ref.append(ref)

    # MPC 求解
    seq = mpc.solve_axis(
        angle,
        rate,
        future_ref
    )

    target_angle = seq[0]

    # 云台动力学
    alpha = 18.0

    rate += alpha * (target_angle - angle) * dt

    rate = np.clip(
        rate,
        -mpc.max_rate,
        mpc.max_rate
    )

    angle += rate * dt

    # 保存数据
    error = np.rad2deg(target - angle)

    times.append(t)
    references.append(np.rad2deg(target))
    gimbal_angles.append(np.rad2deg(angle))
    errors.append(error)
    rates.append(np.rad2deg(rate))

# ==========================================================
# 保存 CSV
# ==========================================================

csv_path = os.path.join(results_dir, "simulation_data.csv")

pd.DataFrame({
    "time": times,
    "target_deg": references,
    "gimbal_deg": gimbal_angles,
    "error_deg": errors,
    "rate_deg_s": rates
}).to_csv(csv_path, index=False)

print(f"CSV saved: {csv_path}")

# ==========================================================
# 创建动画
# ==========================================================

fig = plt.figure(figsize=(14, 7))

# 左边：轨迹
ax1 = plt.subplot(1, 2, 1)

# 右边：误差和角度
ax2 = plt.subplot(2, 2, 2)
ax3 = plt.subplot(2, 2, 4)

# 左图
ax1.set_title("Target Tracking")
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Angle (deg)")
ax1.grid(True)

ax1.set_xlim(0, sim_time)

all_values = references + gimbal_angles

ax1.set_ylim(
    min(all_values) - 10,
    max(all_values) + 10
)

line_target, = ax1.plot([], [], label="Target")
line_gimbal, = ax1.plot([], [], label="Gimbal")

point_target, = ax1.plot([], [], 'o', markersize=8)
point_gimbal, = ax1.plot([], [], 'o', markersize=8)

ax1.legend()

# 右上：误差
ax2.set_title("Tracking Error")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Error (deg)")
ax2.grid(True)

ax2.set_xlim(0, sim_time)
ax2.set_ylim(min(errors) - 5, max(errors) + 5)

line_error, = ax2.plot([], [])

# 右下：云台角度
ax3.set_title("Gimbal Angle")
ax3.set_xlabel("Time (s)")
ax3.set_ylabel("Angle (deg)")
ax3.grid(True)

ax3.set_xlim(0, sim_time)

ax3.set_ylim(
    min(gimbal_angles) - 5,
    max(gimbal_angles) + 5
)

line_angle, = ax3.plot([], [])


def update(frame):

    t = times[:frame]

    target_data = references[:frame]
    gimbal_data = gimbal_angles[:frame]
    error_data = errors[:frame]

    # 左图
    line_target.set_data(t, target_data)
    line_gimbal.set_data(t, gimbal_data)

    if frame > 0:

        point_target.set_data(
            [times[frame - 1]],
            [references[frame - 1]]
        )

        point_gimbal.set_data(
            [times[frame - 1]],
            [gimbal_angles[frame - 1]]
        )

    # 右上
    line_error.set_data(t, error_data)

    # 右下
    line_angle.set_data(t, gimbal_data)

    return (
        line_target,
        line_gimbal,
        point_target,
        point_gimbal,
        line_error,
        line_angle
    )


ani = FuncAnimation(
    fig,
    update,
    frames=len(times),
    interval=20,
    blit=True
)

# ==========================================================
# 保存 GIF
# ==========================================================

gif_path = os.path.join(results_dir, "mpc_tracking.gif")

print("Saving GIF...")

ani.save(
    gif_path,
    writer=PillowWriter(fps=30)
)

print(f"GIF saved: {gif_path}")

plt.tight_layout()
plt.show()
