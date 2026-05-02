#include "immkf_predictor/immkf.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace immkf_predictor
{
namespace
{

constexpr double kEpsilon = 1e-9;

Eigen::Matrix3d cvProcessBlock(double dt, double noise)
{
	const double dt2 = dt * dt;
	const double dt3 = dt2 * dt;
	const double dt4 = dt2 * dt2;
	Eigen::Matrix3d block = Eigen::Matrix3d::Zero();
	block(0, 0) = dt4 / 4.0;
	block(0, 1) = dt3 / 2.0;
	block(1, 0) = dt3 / 2.0;
	block(1, 1) = dt2;
	block(2, 2) = 1.0;
	return block * noise;
}

Eigen::Matrix3d caProcessBlock(double dt, double noise)
{
	const double dt2 = dt * dt;
	const double dt3 = dt2 * dt;
	const double dt4 = dt2 * dt2;
	const double dt5 = dt4 * dt;
	Eigen::Matrix3d block = Eigen::Matrix3d::Zero();
	block(0, 0) = dt5 / 20.0;
	block(0, 1) = dt4 / 8.0;
	block(0, 2) = dt3 / 6.0;
	block(1, 0) = dt4 / 8.0;
	block(1, 1) = dt3 / 3.0;
	block(1, 2) = dt2 / 2.0;
	block(2, 0) = dt3 / 6.0;
	block(2, 1) = dt2 / 2.0;
	block(2, 2) = dt;
	return block * noise;
}

Eigen::Matrix3d singerProcessBlock(double dt, double tau, double noise)
{
	const double scaled_noise = noise * (1.0 - std::exp(-2.0 * dt / std::max(tau, kEpsilon)));
	return caProcessBlock(dt, scaled_noise);
}

Eigen::Matrix3d turnProcessBlock(double dt, double noise)
{
	return cvProcessBlock(dt, noise);
}

}  // namespace

void MotionModel::predict(State & state, Covariance & covariance, double dt) const
{
	const Covariance f = jacobian(state, dt);
	state = transition(state, dt);
	covariance = f * covariance * f.transpose() + processNoise(dt);
}

ConstantVelocityModel::ConstantVelocityModel(MotionModelConfig config)
: config_(std::move(config))
{
}

std::string ConstantVelocityModel::name() const
{
	return "constant_velocity";
}

State ConstantVelocityModel::transition(const State & state, double dt) const
{
	(void)config_;
	State next = state;
	next(0) = state(0) + state(2) * dt;
	next(1) = state(1) + state(3) * dt;
	next(2) = state(2);
	next(3) = state(3);
	next(4) = 0.0;
	next(5) = 0.0;
	return next;
}

Covariance ConstantVelocityModel::jacobian(const State & state, double dt) const
{
	(void)state;
	Covariance f = Covariance::Zero();
	f(0, 0) = 1.0;
	f(0, 2) = dt;
	f(1, 1) = 1.0;
	f(1, 3) = dt;
	f(2, 2) = 1.0;
	f(3, 3) = 1.0;
	return f;
}

Covariance ConstantVelocityModel::processNoise(double dt) const
{
	Covariance q = Covariance::Zero();
	const Eigen::Matrix2d block = Eigen::Matrix2d::Identity() * config_.process_noise;
	q.block<2, 2>(0, 0) = block * (dt * dt * dt * dt / 4.0);
	q.block<2, 2>(0, 2) = block * (dt * dt * dt / 2.0);
	q.block<2, 2>(2, 0) = block * (dt * dt * dt / 2.0);
	q.block<2, 2>(2, 2) = block * (dt * dt);
	q(4, 4) = config_.acceleration_noise;
	q(5, 5) = config_.acceleration_noise;
	return q;
}

ConstantAccelerationModel::ConstantAccelerationModel(MotionModelConfig config)
: config_(std::move(config))
{
}

std::string ConstantAccelerationModel::name() const
{
	return "constant_acceleration";
}

State ConstantAccelerationModel::transition(const State & state, double dt) const
{
	State next = state;
	next(0) = state(0) + state(2) * dt + 0.5 * state(4) * dt * dt;
	next(1) = state(1) + state(3) * dt + 0.5 * state(5) * dt * dt;
	next(2) = state(2) + state(4) * dt;
	next(3) = state(3) + state(5) * dt;
	next(4) = state(4);
	next(5) = state(5);
	return next;
}

Covariance ConstantAccelerationModel::jacobian(const State & state, double dt) const
{
	(void)state;
	Covariance f = Covariance::Zero();
	f(0, 0) = 1.0;
	f(0, 2) = dt;
	f(0, 4) = 0.5 * dt * dt;
	f(1, 1) = 1.0;
	f(1, 3) = dt;
	f(1, 5) = 0.5 * dt * dt;
	f(2, 2) = 1.0;
	f(2, 4) = dt;
	f(3, 3) = 1.0;
	f(3, 5) = dt;
	f(4, 4) = 1.0;
	f(5, 5) = 1.0;
	return f;
}

Covariance ConstantAccelerationModel::processNoise(double dt) const
{
	Covariance q = Covariance::Zero();
	const Eigen::Matrix3d block = caProcessBlock(dt, config_.jerk_noise);
	q.block<3, 3>(0, 0) = block;
	q.block<3, 3>(3, 3) = block;
	return q;
}

SingerModel::SingerModel(MotionModelConfig config)
: config_(std::move(config))
{
}

std::string SingerModel::name() const
{
	return "singer";
}

State SingerModel::transition(const State & state, double dt) const
{
	const double alpha = std::exp(-dt / std::max(config_.singer_time_constant, kEpsilon));
	State next = state;
	next(0) = state(0) + state(2) * dt + 0.5 * state(4) * dt * dt;
	next(1) = state(1) + state(3) * dt + 0.5 * state(5) * dt * dt;
	next(2) = state(2) + state(4) * dt;
	next(3) = state(3) + state(5) * dt;
	next(4) = alpha * state(4);
	next(5) = alpha * state(5);
	return next;
}

Covariance SingerModel::jacobian(const State & state, double dt) const
{
	(void)state;
	const double alpha = std::exp(-dt / std::max(config_.singer_time_constant, kEpsilon));
	Covariance f = Covariance::Zero();
	f(0, 0) = 1.0;
	f(0, 2) = dt;
	f(0, 4) = 0.5 * dt * dt;
	f(1, 1) = 1.0;
	f(1, 3) = dt;
	f(1, 5) = 0.5 * dt * dt;
	f(2, 2) = 1.0;
	f(2, 4) = dt;
	f(3, 3) = 1.0;
	f(3, 5) = dt;
	f(4, 4) = alpha;
	f(5, 5) = alpha;
	return f;
}

Covariance SingerModel::processNoise(double dt) const
{
	Covariance q = Covariance::Zero();
	const Eigen::Matrix3d block = singerProcessBlock(dt, config_.singer_time_constant, config_.process_noise);
	q.block<3, 3>(0, 0) = block;
	q.block<3, 3>(3, 3) = block;
	q(4, 4) += config_.acceleration_noise;
	q(5, 5) += config_.acceleration_noise;
	return q;
}

ConstantTurnModel::ConstantTurnModel(MotionModelConfig config)
: config_(std::move(config))
{
}

std::string ConstantTurnModel::name() const
{
	return "constant_turn";
}

State ConstantTurnModel::transition(const State & state, double dt) const
{
	const double omega = config_.turn_rate;
	State next = state;

	if (std::abs(omega) < kEpsilon) {
		next(0) = state(0) + state(2) * dt;
		next(1) = state(1) + state(3) * dt;
		next(2) = state(2);
		next(3) = state(3);
	} else {
		const double theta = omega * dt;
		const double sin_theta = std::sin(theta);
		const double cos_theta = std::cos(theta);
		const double vx = state(2);
		const double vy = state(3);

		next(0) = state(0) + (sin_theta * vx - (1.0 - cos_theta) * vy) / omega;
		next(1) = state(1) + ((1.0 - cos_theta) * vx + sin_theta * vy) / omega;
		next(2) = cos_theta * vx - sin_theta * vy;
		next(3) = sin_theta * vx + cos_theta * vy;
	}

	next(4) = state(4);
	next(5) = state(5);
	return next;
}

Covariance ConstantTurnModel::jacobian(const State & state, double dt) const
{
	(void)state;
	const double omega = config_.turn_rate;
	Covariance f = Covariance::Identity();

	if (std::abs(omega) < kEpsilon) {
		f(0, 2) = dt;
		f(1, 3) = dt;
		return f;
	}

	const double theta = omega * dt;
	const double sin_theta = std::sin(theta);
	const double cos_theta = std::cos(theta);

	f(0, 2) = sin_theta / omega;
	f(0, 3) = -(1.0 - cos_theta) / omega;
	f(1, 2) = (1.0 - cos_theta) / omega;
	f(1, 3) = sin_theta / omega;
	f(2, 2) = cos_theta;
	f(2, 3) = -sin_theta;
	f(3, 2) = sin_theta;
	f(3, 3) = cos_theta;
	return f;
}

Covariance ConstantTurnModel::processNoise(double dt) const
{
	Covariance q = Covariance::Zero();
	const Eigen::Matrix3d block = turnProcessBlock(dt, config_.process_noise);
	q.block<3, 3>(0, 0) = block;
	q.block<3, 3>(3, 3) = block;
	q(4, 4) = config_.acceleration_noise;
	q(5, 5) = config_.acceleration_noise;
	return q;
}

std::unique_ptr<MotionModel> createMotionModel(
	MotionModelType type,
	const MotionModelConfig & config)
{
	switch (type) {
		case MotionModelType::ConstantVelocity:
			return std::make_unique<ConstantVelocityModel>(config);
		case MotionModelType::ConstantAcceleration:
			return std::make_unique<ConstantAccelerationModel>(config);
		case MotionModelType::Singer:
			return std::make_unique<SingerModel>(config);
		case MotionModelType::ConstantTurn:
			return std::make_unique<ConstantTurnModel>(config);
	}

	return std::make_unique<ConstantVelocityModel>(config);
}

MeasurementMatrix ImmkfPredictor::defaultMeasurementMatrix()
{
	MeasurementMatrix h = MeasurementMatrix::Zero();
	h(0, 0) = 1.0;
	h(1, 1) = 1.0;
	return h;
}

TransitionMatrix ImmkfPredictor::defaultTransitionMatrix()
{
	TransitionMatrix matrix = TransitionMatrix::Constant(0.05);
	for (int i = 0; i < matrix.rows(); ++i) {
		matrix(i, i) = 0.85;
	}
	return matrix;
}

ImmkfPredictor::ImmkfPredictor(const ImmkfConfig & config)
: config_(config)
{
	if (config_.transition_matrix.isZero(0)) {
		config_.transition_matrix = defaultTransitionMatrix();
	}

	if (config_.measurement_matrix.isZero(0)) {
		config_.measurement_matrix = defaultMeasurementMatrix();
	}

	models_[0] = createMotionModel(MotionModelType::ConstantVelocity);
	models_[1] = createMotionModel(MotionModelType::ConstantAcceleration);
	models_[2] = createMotionModel(MotionModelType::Singer);
	models_[3] = createMotionModel(MotionModelType::ConstantTurn);

	mode_probabilities_ = config_.initial_probabilities;
	mode_probabilities_[0] = std::clamp(mode_probabilities_[0], kEpsilon, 1.0);
	mode_probabilities_[1] = std::clamp(mode_probabilities_[1], kEpsilon, 1.0);
	mode_probabilities_[2] = std::clamp(mode_probabilities_[2], kEpsilon, 1.0);
	mode_probabilities_[3] = std::clamp(mode_probabilities_[3], kEpsilon, 1.0);
	const double total = std::accumulate(mode_probabilities_.begin(), mode_probabilities_.end(), 0.0);
	for (double & probability : mode_probabilities_) {
		probability /= std::max(total, kEpsilon);
	}

	for (std::size_t i = 0; i < kModelCount; ++i) {
		model_states_[i].setZero();
		model_covariances_[i].setIdentity();
	}

	fused_state_.setZero();
	fused_covariance_.setIdentity();
}

void ImmkfPredictor::reset(const State & state, const Covariance & covariance)
{
	for (std::size_t i = 0; i < kModelCount; ++i) {
		model_states_[i] = state;
		model_covariances_[i] = covariance;
	}

	fused_state_ = state;
	fused_covariance_ = covariance;
	initialized_ = true;
}

void ImmkfPredictor::mixModelStates()
{
	std::array<double, kModelCount> mixed_probabilities{};
	std::array<State, kModelCount> mixed_states{};
	std::array<Covariance, kModelCount> mixed_covariances{};

	for (std::size_t j = 0; j < kModelCount; ++j) {
		double normalization = 0.0;
		for (std::size_t i = 0; i < kModelCount; ++i) {
			normalization += config_.transition_matrix(i, j) * mode_probabilities_[i];
		}

		mixed_probabilities[j] = normalization;
		mixed_states[j].setZero();
		mixed_covariances[j].setZero();

		const double safe_normalization = std::max(normalization, kEpsilon);
		for (std::size_t i = 0; i < kModelCount; ++i) {
			const double weight = config_.transition_matrix(i, j) * mode_probabilities_[i] / safe_normalization;
			mixed_states[j] += weight * model_states_[i];
		}

		for (std::size_t i = 0; i < kModelCount; ++i) {
			const double weight = config_.transition_matrix(i, j) * mode_probabilities_[i] / safe_normalization;
			const State residual = model_states_[i] - mixed_states[j];
			mixed_covariances[j] += weight * (
				model_covariances_[i] + residual * residual.transpose());
		}
	}

	for (std::size_t i = 0; i < kModelCount; ++i) {
		if (mixed_probabilities[i] > 0.0) {
			mode_probabilities_[i] = mixed_probabilities[i];
		}
		model_states_[i] = mixed_states[i];
		model_covariances_[i] = mixed_covariances[i];
	}

	const double total = std::accumulate(mode_probabilities_.begin(), mode_probabilities_.end(), 0.0);
	for (double & probability : mode_probabilities_) {
		probability /= std::max(total, kEpsilon);
	}
}

void ImmkfPredictor::rebuildFusedEstimate()
{
	fused_state_.setZero();
	fused_covariance_.setZero();

	for (std::size_t i = 0; i < kModelCount; ++i) {
		fused_state_ += mode_probabilities_[i] * model_states_[i];
	}

	for (std::size_t i = 0; i < kModelCount; ++i) {
		const State residual = model_states_[i] - fused_state_;
		fused_covariance_ += mode_probabilities_[i] * (
			model_covariances_[i] + residual * residual.transpose());
	}
}

void ImmkfPredictor::predict(double dt)
{
	if (!initialized_) {
		reset(State::Zero(), Covariance::Identity());
	}

	mixModelStates();

	for (std::size_t i = 0; i < kModelCount; ++i) {
		models_[i]->predict(model_states_[i], model_covariances_[i], dt);
	}

	rebuildFusedEstimate();
}

double ImmkfPredictor::measurementLikelihood(
	std::size_t index,
	const Measurement & measurement) const
{
	const MeasurementMatrix & h = config_.measurement_matrix;
	const Eigen::Vector2d innovation = measurement - h * model_states_[index];
	const Eigen::Matrix2d innovation_covariance =
		h * model_covariances_[index] * h.transpose() + config_.measurement_noise;

	const double determinant = std::max(innovation_covariance.determinant(), kEpsilon);
	const Eigen::Matrix2d inverse = innovation_covariance.inverse();
	const double exponent = -0.5 * innovation.transpose() * inverse * innovation;
	const double normalizer = 1.0 / (2.0 * M_PI * std::sqrt(determinant));
	return normalizer * std::exp(exponent);
}

void ImmkfPredictor::update(const Measurement & measurement)
{
	if (!initialized_) {
		reset(State::Zero(), Covariance::Identity());
	}

	double probability_sum = 0.0;
	for (std::size_t i = 0; i < kModelCount; ++i) {
		const MeasurementMatrix & h = config_.measurement_matrix;
		const Eigen::Vector2d innovation = measurement - h * model_states_[i];
		const Eigen::Matrix2d s = h * model_covariances_[i] * h.transpose() + config_.measurement_noise;
		const Eigen::Matrix<double, 6, 2> kalman_gain =
			model_covariances_[i] * h.transpose() * s.inverse();
		model_states_[i] = model_states_[i] + kalman_gain * innovation;

		const Covariance identity = Covariance::Identity();
		model_covariances_[i] = (identity - kalman_gain * h) * model_covariances_[i];

		const double likelihood = measurementLikelihood(i, measurement);
		mode_probabilities_[i] *= likelihood;
		probability_sum += mode_probabilities_[i];
	}

	for (double & probability : mode_probabilities_) {
		probability /= std::max(probability_sum, kEpsilon);
	}

	rebuildFusedEstimate();
}

std::vector<State> ImmkfPredictor::predictTrajectory(double dt, std::size_t horizon) const
{
	std::vector<State> trajectory;
	trajectory.reserve(horizon);

	if (!initialized_) {
		return trajectory;
	}

	std::array<State, kModelCount> model_states = model_states_;
	std::array<Covariance, kModelCount> model_covariances = model_covariances_;
	std::array<double, kModelCount> mode_probabilities = mode_probabilities_;
	State fused_state = fused_state_;

	for (std::size_t step = 0; step < horizon; ++step) {
		std::array<State, kModelCount> mixed_states{};
		std::array<Covariance, kModelCount> mixed_covariances{};

		for (std::size_t j = 0; j < kModelCount; ++j) {
			double normalization = 0.0;
			for (std::size_t i = 0; i < kModelCount; ++i) {
				normalization += config_.transition_matrix(i, j) * mode_probabilities[i];
			}

			const double safe_normalization = std::max(normalization, kEpsilon);
			mixed_states[j].setZero();
			mixed_covariances[j].setZero();

			for (std::size_t i = 0; i < kModelCount; ++i) {
				const double weight = config_.transition_matrix(i, j) * mode_probabilities[i] / safe_normalization;
				mixed_states[j] += weight * model_states[i];
			}

			for (std::size_t i = 0; i < kModelCount; ++i) {
				const double weight = config_.transition_matrix(i, j) * mode_probabilities[i] / safe_normalization;
				const State residual = model_states[i] - mixed_states[j];
				mixed_covariances[j] += weight * (model_covariances[i] + residual * residual.transpose());
			}
		}

		for (std::size_t i = 0; i < kModelCount; ++i) {
			model_states[i] = mixed_states[i];
			model_covariances[i] = mixed_covariances[i];
		}

		const double probability_total = std::accumulate(mode_probabilities.begin(), mode_probabilities.end(), 0.0);
		for (double & probability : mode_probabilities) {
			probability /= std::max(probability_total, kEpsilon);
		}

		for (std::size_t i = 0; i < kModelCount; ++i) {
			models_[i]->predict(model_states[i], model_covariances[i], dt);
		}

		fused_state.setZero();
		for (std::size_t i = 0; i < kModelCount; ++i) {
			fused_state += mode_probabilities[i] * model_states[i];
		}

		trajectory.push_back(fused_state);
	}

	return trajectory;
}

const State & ImmkfPredictor::fusedState() const
{
	return fused_state_;
}

const Covariance & ImmkfPredictor::fusedCovariance() const
{
	return fused_covariance_;
}

std::array<double, 4> ImmkfPredictor::modeProbabilities() const
{
	return mode_probabilities_;
}

bool ImmkfPredictor::isInitialized() const
{
	return initialized_;
}

TrackManager::TrackManager(const ImmkfConfig & config)
: config_(config)
{
}

std::shared_ptr<ImmkfPredictor> TrackManager::getOrCreateTrack(const std::string & track_id)
{
	auto it = tracks_.find(track_id);
	if (it != tracks_.end()) {
		return it->second;
	}

	auto predictor = std::make_shared<ImmkfPredictor>(config_);
	tracks_[track_id] = predictor;
	last_update_times_[track_id] = 0.0;
	return predictor;
}

void TrackManager::predictAll(double dt)
{
	for (auto & [track_id, predictor] : tracks_) {
		predictor->predict(dt);
	}
}

void TrackManager::updateTrack(const std::string & track_id, const Measurement & measurement)
{
	auto predictor = getOrCreateTrack(track_id);
	predictor->update(measurement);
	last_update_times_[track_id] = 0.0;
}

std::vector<State> TrackManager::getTrajectory(
	const std::string & track_id,
	double dt,
	std::size_t horizon) const
{
	auto it = tracks_.find(track_id);
	if (it == tracks_.end()) {
		return std::vector<State>{};
	}

	return it->second->predictTrajectory(dt, horizon);
}

std::vector<TrackState> TrackManager::getAllTracks() const
{
	std::vector<TrackState> result;
	for (const auto & [track_id, predictor] : tracks_) {
		TrackState ts;
		ts.track_id = track_id;
		ts.predictor = predictor;
		ts.trajectory = predictor->predictTrajectory(0.1, 10);  // Default horizon for display
		auto it = last_update_times_.find(track_id);
		ts.last_update_time = (it != last_update_times_.end()) ? it->second : 0.0;
		result.push_back(ts);
	}
	return result;
}

void TrackManager::pruneInactiveTracks(double current_time, double timeout_seconds)
{
	std::vector<std::string> to_remove;
	for (const auto & [track_id, last_time] : last_update_times_) {
		if (current_time - last_time > timeout_seconds) {
			to_remove.push_back(track_id);
		}
	}

	for (const auto & track_id : to_remove) {
		tracks_.erase(track_id);
		last_update_times_.erase(track_id);
	}
}

void TrackManager::clear()
{
	tracks_.clear();
	last_update_times_.clear();
}

std::size_t TrackManager::size() const
{
	return tracks_.size();
}

}  // namespace immkf_predictor

