#include "mpc_gimbal_planner/mpc_view_planner.h"
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>

namespace mpc_gimbal_planner {

MPCViewPlanner::MPCViewPlanner(int horizon, double dt)
    : horizon_(horizon), dt_(dt),
      w_tracking_(1.0), w_smoothness_(0.5), w_control_(0.2),
      max_pan_rate_(2.0), max_tilt_rate_(2.0),
      max_pan_accel_(1.0), max_tilt_accel_(1.0),
      fx_(1470.0), fy_(1470.0), cx_(480.0), cy_(360.0),
      img_width_(960), img_height_(720) {
}

void MPCViewPlanner::updateTrajectory(const std::vector<TrajectoryPoint>& trajectory) {
    trajectory_ = trajectory;
}

double MPCViewPlanner::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double MPCViewPlanner::angleDifference(double a1, double a2) {
    double diff = normalizeAngle(a1 - a2);
    return diff;
}

double MPCViewPlanner::saturate(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(max_val, value));
}

void MPCViewPlanner::projectPoint3D(double x, double y, double z,
                                   double& pan, double& tilt) const {
    // Pan: yaw angle (rotation around z-axis in world frame)
    pan = std::atan2(y, x);

    // Tilt: pitch angle (elevation angle, negative z is downward)
    double horizontal_dist = std::sqrt(x * x + y * y);
    tilt = std::atan2(-z, horizontal_dist);

    pan = normalizeAngle(pan);
    tilt = normalizeAngle(tilt);
}

void MPCViewPlanner::computeDesiredAngles(std::vector<double>& desired_pan,
                                         std::vector<double>& desired_tilt) const {
    desired_pan.clear();
    desired_tilt.clear();

    for (int i = 0; i < horizon_ && i < (int)trajectory_.size(); ++i) {
        double pan, tilt;
        projectPoint3D(trajectory_[i].x, trajectory_[i].y, trajectory_[i].z, pan, tilt);
        desired_pan.push_back(pan);
        desired_tilt.push_back(tilt);
    }

    // Extend with last value if trajectory shorter than horizon
    if (desired_pan.size() < (size_t)horizon_) {
        double last_pan = desired_pan.empty() ? 0.0 : desired_pan.back();
        double last_tilt = desired_tilt.empty() ? 0.0 : desired_tilt.back();
        while (desired_pan.size() < (size_t)horizon_) {
            desired_pan.push_back(last_pan);
            desired_tilt.push_back(last_tilt);
        }
    }
}

GimbalCommand MPCViewPlanner::solve(double current_pan, double current_tilt) {
    if (trajectory_.empty()) {
        GimbalCommand cmd;
        cmd.pan = current_pan;
        cmd.tilt = current_tilt;
        cmd.pan_rate = 0.0;
        cmd.tilt_rate = 0.0;
        cmd.timestamp = 0.0;
        return cmd;
    }

    // Initialize control history
    if (pan_history_.empty()) {
        pan_history_.push_back(current_pan);
        tilt_history_.push_back(current_tilt);
    }

    // Compute desired angles for prediction horizon
    std::vector<double> desired_pan(horizon_);
    std::vector<double> desired_tilt(horizon_);
    computeDesiredAngles(desired_pan, desired_tilt);

    double prev_pan = pan_history_.back();
    double prev_tilt = tilt_history_.back();

    // Compute previous velocities and accelerations
    double prev_pan_rate = 0.0, prev_tilt_rate = 0.0;
    if (pan_history_.size() >= 2) {
        prev_pan_rate = angleDifference(pan_history_[pan_history_.size()-1],
                                       pan_history_[pan_history_.size()-2]) / dt_;
        prev_tilt_rate = angleDifference(tilt_history_[tilt_history_.size()-1],
                                        tilt_history_[tilt_history_.size()-2]) / dt_;
    }

    // ========== Build MPC Quadratic Program ==========
    // State vector: [pan_0, tilt_0, pan_1, tilt_1, ..., pan_N, tilt_N]
    // N = horizon - 1
    // Variables: x = [u_pan_0, u_tilt_0, ..., u_pan_N, u_tilt_N] where u is angle at step k

    int n_vars = 2 * horizon_;

    // Hessian matrix
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_vars, n_vars);
    // Gradient vector
    Eigen::VectorXd g = Eigen::VectorXd::Zero(n_vars);

    // Cost 1: Tracking error - w_tracking * sum||u_k - u_desired_k||^2
    for (int k = 0; k < horizon_; ++k) {
        int idx_pan = 2 * k;
        int idx_tilt = 2 * k + 1;

        H(idx_pan, idx_pan) += 2.0 * w_tracking_;
        H(idx_tilt, idx_tilt) += 2.0 * w_tracking_;
        g(idx_pan) -= 2.0 * w_tracking_ * desired_pan[k];
        g(idx_tilt) -= 2.0 * w_tracking_ * desired_tilt[k];
    }

    // Cost 2: Smoothness - w_smoothness * sum||u_k - u_{k-1}||^2
    for (int k = 0; k < horizon_; ++k) {
        int idx_pan = 2 * k;
        int idx_tilt = 2 * k + 1;

        if (k == 0) {
            // First step compared to previous state
            H(idx_pan, idx_pan) += 2.0 * w_smoothness_;
            H(idx_tilt, idx_tilt) += 2.0 * w_smoothness_;
            g(idx_pan) -= 2.0 * w_smoothness_ * prev_pan;
            g(idx_tilt) -= 2.0 * w_smoothness_ * prev_tilt;
        } else {
            // Subsequent steps
            int prev_idx_pan = 2 * (k - 1);
            int prev_idx_tilt = 2 * (k - 1) + 1;

            H(idx_pan, idx_pan) += 2.0 * w_smoothness_;
            H(idx_pan, prev_idx_pan) -= 2.0 * w_smoothness_;
            H(prev_idx_pan, idx_pan) -= 2.0 * w_smoothness_;
            H(prev_idx_pan, prev_idx_pan) += 2.0 * w_smoothness_;
        }
    }

    // Cost 3: Control effort (acceleration regularization) - w_control * sum||a_k||^2
    // where a_k = (u_k - u_{k-1}) - (u_{k-1} - u_{k-2})
    for (int k = 1; k < horizon_; ++k) {
        int idx_pan = 2 * k;
        int idx_tilt = 2 * k + 1;
        int prev_idx_pan = 2 * (k - 1);
        int prev_idx_tilt = 2 * (k - 1) + 1;

        if (k == 1) {
            // a_1 = (u_1 - u_0) - (u_0 - u_prev)
            H(idx_pan, idx_pan) += 2.0 * w_control_;
            H(idx_pan, prev_idx_pan) -= 4.0 * w_control_;
            H(prev_idx_pan, idx_pan) -= 4.0 * w_control_;
            H(prev_idx_pan, prev_idx_pan) += 2.0 * w_control_;

            g(idx_pan) -= 2.0 * w_control_ * (prev_pan - prev_pan_rate * dt_);
            g(prev_idx_pan) += 4.0 * w_control_ * (prev_pan - prev_pan_rate * dt_);

            H(idx_tilt, idx_tilt) += 2.0 * w_control_;
            H(idx_tilt, prev_idx_tilt) -= 4.0 * w_control_;
            H(prev_idx_tilt, idx_tilt) -= 4.0 * w_control_;
            H(prev_idx_tilt, prev_idx_tilt) += 2.0 * w_control_;

            g(idx_tilt) -= 2.0 * w_control_ * (prev_tilt - prev_tilt_rate * dt_);
            g(prev_idx_tilt) += 4.0 * w_control_ * (prev_tilt - prev_tilt_rate * dt_);
        } else {
            // General case: a_k = (u_k - u_{k-1}) - (u_{k-1} - u_{k-2})
            int prev_prev_idx_pan = 2 * (k - 2);
            int prev_prev_idx_tilt = 2 * (k - 2) + 1;

            H(idx_pan, idx_pan) += 2.0 * w_control_;
            H(idx_pan, prev_idx_pan) -= 4.0 * w_control_;
            H(idx_pan, prev_prev_idx_pan) += 2.0 * w_control_;
            H(prev_idx_pan, idx_pan) -= 4.0 * w_control_;
            H(prev_idx_pan, prev_idx_pan) += 8.0 * w_control_;
            H(prev_idx_pan, prev_prev_idx_pan) -= 4.0 * w_control_;
            H(prev_prev_idx_pan, idx_pan) += 2.0 * w_control_;
            H(prev_prev_idx_pan, prev_idx_pan) -= 4.0 * w_control_;
            H(prev_prev_idx_pan, prev_prev_idx_pan) += 2.0 * w_control_;

            H(idx_tilt, idx_tilt) += 2.0 * w_control_;
            H(idx_tilt, prev_idx_tilt) -= 4.0 * w_control_;
            H(idx_tilt, prev_prev_idx_tilt) += 2.0 * w_control_;
            H(prev_idx_tilt, idx_tilt) -= 4.0 * w_control_;
            H(prev_idx_tilt, prev_idx_tilt) += 8.0 * w_control_;
            H(prev_idx_tilt, prev_prev_idx_tilt) -= 4.0 * w_control_;
            H(prev_prev_idx_tilt, idx_tilt) += 2.0 * w_control_;
            H(prev_prev_idx_tilt, prev_idx_tilt) -= 4.0 * w_control_;
            H(prev_prev_idx_tilt, prev_prev_idx_tilt) += 2.0 * w_control_;
        }
    }

    // Solve unconstrained QP: min 0.5 * x^T * H * x + g^T * x
    // Solution: x* = -H^{-1} * g
    Eigen::VectorXd x_opt = H.ldlt().solve(-g);

    // Extract optimal first step
    double opt_pan = x_opt(0);
    double opt_tilt = x_opt(1);

    // Apply rate constraints
    double pan_delta = angleDifference(opt_pan, prev_pan);
    double tilt_delta = angleDifference(opt_tilt, prev_tilt);

    pan_delta = saturate(pan_delta, -max_pan_rate_ * dt_, max_pan_rate_ * dt_);
    tilt_delta = saturate(tilt_delta, -max_tilt_rate_ * dt_, max_tilt_rate_ * dt_);

    double pan_rate = pan_delta / dt_;
    double tilt_rate = tilt_delta / dt_;

    // Apply acceleration constraints
    double pan_accel_desired = (pan_rate - prev_pan_rate) / dt_;
    double tilt_accel_desired = (tilt_rate - prev_tilt_rate) / dt_;

    pan_accel_desired = saturate(pan_accel_desired, -max_pan_accel_, max_pan_accel_);
    tilt_accel_desired = saturate(tilt_accel_desired, -max_tilt_accel_, max_tilt_accel_);

    pan_rate = prev_pan_rate + pan_accel_desired * dt_;
    tilt_rate = prev_tilt_rate + tilt_accel_desired * dt_;

    // Final command
    double next_pan = prev_pan + pan_rate * dt_;
    double next_tilt = prev_tilt + tilt_rate * dt_;

    // Update history for next iteration
    pan_history_.push_back(next_pan);
    tilt_history_.push_back(next_tilt);

    // Limit history size to avoid memory growth
    if (pan_history_.size() > 100) {
        pan_history_.erase(pan_history_.begin());
        tilt_history_.erase(tilt_history_.begin());
    }

    GimbalCommand cmd;
    cmd.pan = next_pan;
    cmd.tilt = next_tilt;
    cmd.pan_rate = pan_rate;
    cmd.tilt_rate = tilt_rate;
    cmd.timestamp = trajectory_[0].timestamp;

    return cmd;
}

}  // namespace mpc_gimbal_planner
