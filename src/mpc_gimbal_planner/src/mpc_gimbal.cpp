#include "mpc_gimbal_planner/mpc_gimbal.hpp"

#include <algorithm>
#include <cmath>

namespace mpc_gimbal_planner
{
MPCGimbal::MPCGimbal(const Config & config)
: config_(config)
{
}

const MPCGimbal::Config & MPCGimbal::config() const
{
	return config_;
}

double MPCGimbal::clamp(double value, double min_value, double max_value)
{
	return std::min(std::max(value, min_value), max_value);
}

std::vector<double> MPCGimbal::expandReference(
	const std::vector<double> & reference_sequence) const
{
	std::vector<double> expanded;
	expanded.reserve(config_.horizon_steps);

	if (reference_sequence.empty()) {
		expanded.assign(config_.horizon_steps, 0.0);
		return expanded;
	}

	for (std::size_t i = 0; i < config_.horizon_steps; ++i) {
		const std::size_t index = std::min(i, reference_sequence.size() - 1);
		expanded.push_back(reference_sequence[index]);
	}

	return expanded;
}

std::vector<double> MPCGimbal::projectSequence(
	const std::vector<double> & sequence,
	double current_angle,
	double min_angle,
	double max_angle) const
{
	std::vector<double> projected = sequence;
	const double max_delta = config_.max_rate * config_.prediction_dt;

	for (int iteration = 0; iteration < 4; ++iteration) {
		double previous_angle = current_angle;
		for (double & angle : projected) {
			angle = clamp(angle, min_angle, max_angle);
			angle = clamp(angle, previous_angle - max_delta, previous_angle + max_delta);
			previous_angle = angle;
		}
	}

	return projected;
}

std::vector<double> MPCGimbal::solveAxis(
	double current_angle,
	double current_rate,
	const std::vector<double> & reference_sequence,
	double min_angle,
	double max_angle) const
{
	const std::size_t horizon = config_.horizon_steps;
	if (horizon == 0) {
		return {};
	}

	const auto references = expandReference(reference_sequence);
	Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(horizon, horizon);
	Eigen::VectorXd gradient = Eigen::VectorXd::Zero(horizon);

	auto addQuadraticTerm = [&horizon](
			Eigen::MatrixXd & h,
			Eigen::VectorXd & g,
			const std::vector<std::pair<std::size_t, double>> & coefficients,
			double constant,
			double weight) {
		for (const auto & [row, coeff_row] : coefficients) {
			for (const auto & [col, coeff_col] : coefficients) {
				h(row, col) += 2.0 * weight * coeff_row * coeff_col;
			}
			g(row) += -2.0 * weight * constant * coeff_row;
		}
	};

	for (std::size_t i = 0; i < horizon; ++i) {
		const double tracking_weight = config_.track_weight;
		hessian(i, i) += 2.0 * tracking_weight;
		gradient(i) += -2.0 * tracking_weight * references[i];
	}

	for (std::size_t i = 0; i < horizon; ++i) {
		const double smooth_weight = config_.smooth_weight;
		if (i == 0) {
			hessian(0, 0) += 2.0 * smooth_weight;
			gradient(0) += -2.0 * smooth_weight * current_angle;
			continue;
		}

		hessian(i, i) += 2.0 * smooth_weight;
		hessian(i - 1, i - 1) += 2.0 * smooth_weight;
		hessian(i, i - 1) += -2.0 * smooth_weight;
		hessian(i - 1, i) += -2.0 * smooth_weight;
	}

	const double control_weight = config_.control_weight / (config_.prediction_dt * config_.prediction_dt);
	if (horizon >= 1) {
		addQuadraticTerm(
			hessian, gradient,
			{{0, 1.0}},
			current_angle + config_.prediction_dt * current_rate,
			control_weight);
	}
	if (horizon >= 2) {
		addQuadraticTerm(
			hessian, gradient,
			{{1, 1.0}, {0, -2.0}},
			current_angle,
			control_weight);
	}
	for (std::size_t i = 2; i < horizon; ++i) {
		addQuadraticTerm(
			hessian, gradient,
			{{i, 1.0}, {i - 1, -2.0}, {i - 2, 1.0}},
			0.0,
			control_weight);
	}

	hessian += 1e-6 * Eigen::MatrixXd::Identity(horizon, horizon);

	std::vector<double> planned_sequence(horizon, current_angle);
	Eigen::LDLT<Eigen::MatrixXd> solver(hessian);
	if (solver.info() == Eigen::Success) {
		const Eigen::VectorXd solution = solver.solve(-gradient);
		if (solver.info() == Eigen::Success) {
			for (std::size_t i = 0; i < horizon; ++i) {
				planned_sequence[i] = solution(static_cast<Eigen::Index>(i));
			}
		}
	}

	return projectSequence(planned_sequence, current_angle, min_angle, max_angle);
}

MPCGimbal::Solution MPCGimbal::solve(
	const Eigen::Vector2d & current_angles,
	const Eigen::Vector2d & current_rates,
	const std::vector<Eigen::Vector2d> & reference_sequence) const
{
	Solution solution;

	if (reference_sequence.empty()) {
		solution.command = current_angles;
		solution.rate = Eigen::Vector2d::Zero();
		return solution;
	}

	std::vector<double> yaw_references;
	std::vector<double> pitch_references;
	yaw_references.reserve(reference_sequence.size());
	pitch_references.reserve(reference_sequence.size());
	for (const auto & reference : reference_sequence) {
		yaw_references.push_back(reference.x());
		pitch_references.push_back(reference.y());
	}

	const auto yaw_sequence = solveAxis(
		current_angles.x(), current_rates.x(), yaw_references,
		config_.yaw_min, config_.yaw_max);
	const auto pitch_sequence = solveAxis(
		current_angles.y(), current_rates.y(), pitch_references,
		config_.pitch_min, config_.pitch_max);

	const double yaw_target = yaw_sequence.empty() ? current_angles.x() : yaw_sequence.front();
	const double pitch_target = pitch_sequence.empty() ? current_angles.y() : pitch_sequence.front();

	solution.command.x() = yaw_target;
	solution.command.y() = pitch_target;
	solution.rate.x() = (yaw_target - current_angles.x()) / config_.prediction_dt;
	solution.rate.y() = (pitch_target - current_angles.y()) / config_.prediction_dt;

	solution.trajectory.reserve(config_.horizon_steps);
	for (std::size_t i = 0; i < config_.horizon_steps; ++i) {
		const std::size_t yaw_index = yaw_sequence.empty() ? 0 : std::min(i, yaw_sequence.size() - 1);
		const std::size_t pitch_index = pitch_sequence.empty() ? 0 : std::min(i, pitch_sequence.size() - 1);
		solution.trajectory.emplace_back(
			yaw_sequence.empty() ? current_angles.x() : yaw_sequence[yaw_index],
			pitch_sequence.empty() ? current_angles.y() : pitch_sequence[pitch_index]);
	}

	return solution;
}

} // namespace mpc_gimbal_planner