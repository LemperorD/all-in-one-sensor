#ifndef MPC_GIMBAL_HPP_
#define MPC_GIMBAL_HPP_

#include <Eigen/Dense>

#include <cstddef>
#include <vector>

namespace mpc_gimbal_planner
{

class MPCGimbal {
public:
  struct Config
  {
    std::size_t horizon_steps{10};
    double prediction_dt{0.1};
    double track_weight{1.0};
    double smooth_weight{0.2};
    double control_weight{0.05};
    double yaw_min{-1.57};
    double yaw_max{1.57};
    double pitch_min{-0.8};
    double pitch_max{0.8};
    double max_rate{1.0};
  };

  struct Solution
  {
    Eigen::Vector2d command{Eigen::Vector2d::Zero()};
    Eigen::Vector2d rate{Eigen::Vector2d::Zero()};
    std::vector<Eigen::Vector2d> trajectory;
  };

  explicit MPCGimbal(const Config & config);
  ~MPCGimbal() = default;

  Solution solve(
    const Eigen::Vector2d & current_angles,
    const Eigen::Vector2d & current_rates,
    const std::vector<Eigen::Vector2d> & reference_sequence) const;

  const Config & config() const;

private:
  std::vector<double> solveAxis(
    double current_angle,
    double current_rate,
    const std::vector<double> & reference_sequence,
    double min_angle,
    double max_angle) const;

  std::vector<double> expandReference(
    const std::vector<double> & reference_sequence) const;

  std::vector<double> projectSequence(
    const std::vector<double> & sequence,
    double current_angle,
    double min_angle,
    double max_angle) const;

  static double clamp(double value, double min_value, double max_value);

  Config config_;

};

} // namespace mpc_gimbal_planner

#endif // MPC_GIMBAL_HPP_