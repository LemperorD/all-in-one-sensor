#ifndef MPC_VIEW_PLANNER_H_
#define MPC_VIEW_PLANNER_H_

#include <vector>
#include <Eigen/Dense>
#include <cmath>

namespace mpc_gimbal_planner {

/**
 * Predicted trajectory point from IMMKF predictor
 */
struct TrajectoryPoint {
    double timestamp;
    double x, y, z;           // 3D position
    double vx, vy, vz;        // 3D velocity
    double confidence;         // prediction confidence
};

/**
 * PTZ (Pan-Tilt-Zoom) gimbal command output
 */
struct GimbalCommand {
    double pan;               // yaw angle (radians)
    double tilt;              // pitch angle (radians)
    double pan_rate;          // angular velocity (rad/s)
    double tilt_rate;         // angular velocity (rad/s)
    double timestamp;         // command timestamp
};

/**
 * MPC-based Active Gimbal View Planner
 *
 * Cost function with three terms:
 *   J = w_track * ||tracking_error||^2           (view tracking)
 *     + w_smooth * ||angular_velocity||^2        (view smoothness)
 *     + w_control * ||angular_acceleration||^2   (control effort reduction)
 */
class MPCViewPlanner {
public:
    explicit MPCViewPlanner(int horizon = 10, double dt = 0.1);
    ~MPCViewPlanner() = default;

    /**
     * Update with predicted trajectory from IMMKF
     */
    void updateTrajectory(const std::vector<TrajectoryPoint>& trajectory);

    /**
     * Solve MPC to generate optimal gimbal command
     * @param current_pan: current pan angle (radians)
     * @param current_tilt: current tilt angle (radians)
     * @return: gimbal command
     */
    GimbalCommand solve(double current_pan, double current_tilt);

    /**
     * Set cost function weights
     */
    void setWeights(double w_track, double w_smooth, double w_control) {
        w_tracking_ = w_track;
        w_smoothness_ = w_smooth;
        w_control_ = w_control;
    }

    /**
     * Set control constraints
     */
    void setConstraints(double max_pan_rate, double max_tilt_rate,
                       double max_pan_accel, double max_tilt_accel) {
        max_pan_rate_ = max_pan_rate;
        max_tilt_rate_ = max_tilt_rate;
        max_pan_accel_ = max_pan_accel;
        max_tilt_accel_ = max_tilt_accel;
    }

    /**
     * Set camera parameters
     */
    void setCameraParams(double fx, double fy, double cx, double cy,
                        int img_width, int img_height) {
        fx_ = fx;
        fy_ = fy;
        cx_ = cx;
        cy_ = cy;
        img_width_ = img_width;
        img_height_ = img_height;
    }

    // Getters
    int getHorizon() const { return horizon_; }
    double getDt() const { return dt_; }

private:
    // MPC parameters
    int horizon_;              // prediction horizon steps
    double dt_;                // control period (seconds)

    // Cost function weights
    double w_tracking_;        // tracking error weight
    double w_smoothness_;      // smoothness weight
    double w_control_;         // control effort weight

    // Control constraints
    double max_pan_rate_;      // max pan angular velocity (rad/s)
    double max_tilt_rate_;     // max tilt angular velocity (rad/s)
    double max_pan_accel_;     // max pan angular acceleration (rad/s^2)
    double max_tilt_accel_;    // max tilt angular acceleration (rad/s^2)

    // Camera parameters
    double fx_, fy_, cx_, cy_;
    int img_width_, img_height_;

    // Predicted trajectory
    std::vector<TrajectoryPoint> trajectory_;

    // State history for smoothness tracking
    std::vector<double> pan_history_;
    std::vector<double> tilt_history_;

    // Helper functions
    void projectPoint3D(double x, double y, double z,
                       double& pan, double& tilt) const;

    void computeDesiredAngles(std::vector<double>& desired_pan,
                             std::vector<double>& desired_tilt) const;

    static double normalizeAngle(double angle);
    static double angleDifference(double a1, double a2);
    static double saturate(double value, double min_val, double max_val);
};

}  // namespace mpc_gimbal_planner

#endif  // MPC_VIEW_PLANNER_H_
