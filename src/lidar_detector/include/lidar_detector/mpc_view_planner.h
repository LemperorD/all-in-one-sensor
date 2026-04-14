#ifndef MPC_VIEW_PLANNER_H_
#define MPC_VIEW_PLANNER_H_

#include <vector>
#include <Eigen/Dense>
#include <cmath>

namespace lidar_detector {

/**
 * Structure for predicted trajectory point
 * Expected from IMMKF predictor
 */
struct TrajectoryPoint {
    double timestamp;
    double x, y, z;           // 3D position
    double vx, vy, vz;        // 3D velocity
    double confidence;         // prediction confidence
};

/**
 * Structure for PTZ (Pan-Tilt-Zoom) command output
 */
struct PTZCommand {
    double pan;               // yaw angle (radians)
    double tilt;              // pitch angle (radians)
    double pan_rate;          // angular velocity (rad/s)
    double tilt_rate;         // angular velocity (rad/s)
    double timestamp;         // command timestamp
};

/**
 * MPC-based Active View Planner
 * Generates PTZ commands to keep detected objects in optimal view
 *
 * Cost function:
 *   J = w_track * ||tracking_error||^2
 *     + w_smooth * ||angular_velocity||^2
 *     + w_control * ||angular_acceleration||^2
 */
class MPCViewPlanner {
public:
    explicit MPCViewPlanner(int horizon = 10, double dt = 0.1);
    ~MPCViewPlanner() = default;

    /**
     * Update with predicted trajectory from IMMKF
     * @param trajectory: predicted trajectory points
     */
    void updateTrajectory(const std::vector<TrajectoryPoint>& trajectory);

    /**
     * Plan optimal PTZ command for current step
     * @param current_pan: current pan angle (radians)
     * @param current_tilt: current tilt angle (radians)
     * @return: PTZ command
     */
    PTZCommand solve(double current_pan, double current_tilt);

    /**
     * Set cost function weights
     * @param w_track: tracking error weight
     * @param w_smooth: smoothness weight
     * @param w_control: control effort weight
     */
    void setWeights(double w_track, double w_smooth, double w_control) {
        w_tracking_ = w_track;
        w_smoothness_ = w_smooth;
        w_control_ = w_control;
    }

    /**
     * Set control constraints
     * @param max_pan_rate: maximum pan angular velocity (rad/s)
     * @param max_tilt_rate: maximum tilt angular velocity (rad/s)
     * @param max_pan_accel: maximum pan angular acceleration (rad/s^2)
     * @param max_tilt_accel: maximum tilt angular acceleration (rad/s^2)
     */
    void setConstraints(double max_pan_rate, double max_tilt_rate,
                       double max_pan_accel, double max_tilt_accel) {
        max_pan_rate_ = max_pan_rate;
        max_tilt_rate_ = max_tilt_rate;
        max_pan_accel_ = max_pan_accel;
        max_tilt_accel_ = max_tilt_accel;
    }

    /**
     * Set camera parameters for projection
     * @param fx, fy: focal lengths (pixels)
     * @param cx, cy: principal point (pixels)
     * @param img_width, img_height: image dimensions (pixels)
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

    // State history for smoothness
    std::vector<double> pan_history_;
    std::vector<double> tilt_history_;

    // Helper functions
    /**
     * Project 3D point to pan-tilt angles
     * Pan: yaw angle relative to camera z-axis
     * Tilt: pitch angle relative to camera horizontal plane
     */
    void projectPoint3D(double x, double y, double z,
                       double& pan, double& tilt) const;

    /**
     * Compute desired pan-tilt angles for each step in horizon
     */
    void computeDesiredAngles(std::vector<double>& desired_pan,
                             std::vector<double>& desired_tilt) const;

    /**
     * Clamp angle to [-pi, pi]
     */
    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    /**
     * Compute angle difference with consideration of wrapping
     */
    static double angleDifference(double a1, double a2) {
        double diff = normalizeAngle(a1 - a2);
        return diff;
    }

    /**
     * Apply constraint with saturation
     */
    static double saturate(double value, double min_val, double max_val) {
        return std::max(min_val, std::min(max_val, value));
    }
};

}  // namespace lidar_detector

#endif  // MPC_VIEW_PLANNER_H_
