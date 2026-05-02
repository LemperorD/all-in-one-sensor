#ifndef IMMKF_PREDICTOR_IMMKF_HPP
#define IMMKF_PREDICTOR_IMMKF_HPP

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

namespace immkf_predictor
{

using State = Eigen::Matrix<double, 6, 1>;
using Covariance = Eigen::Matrix<double, 6, 6>;
using Measurement = Eigen::Matrix<double, 2, 1>;
using MeasurementMatrix = Eigen::Matrix<double, 2, 6>;
using TransitionMatrix = Eigen::Matrix<double, 4, 4>;

enum class MotionModelType
{
	ConstantVelocity = 0,
	ConstantAcceleration = 1,
	Singer = 2,
	ConstantTurn = 3,
};

struct MotionModelConfig
{
	double process_noise = 1.0;
	double acceleration_noise = 1.0;
	double jerk_noise = 1.0;
	double singer_time_constant = 2.0;
	double turn_rate = 0.2;
};

struct ImmkfConfig
{
	TransitionMatrix transition_matrix = TransitionMatrix::Zero();
	std::array<double, 4> initial_probabilities{{0.25, 0.25, 0.25, 0.25}};
	MeasurementMatrix measurement_matrix = MeasurementMatrix::Zero();
	Eigen::Matrix2d measurement_noise = Eigen::Matrix2d::Identity();
};

class MotionModel
{
public:
	virtual ~MotionModel() = default;

	virtual std::string name() const = 0;
	virtual State transition(const State & state, double dt) const = 0;
	virtual Covariance jacobian(const State & state, double dt) const = 0;
	virtual Covariance processNoise(double dt) const = 0;

	void predict(State & state, Covariance & covariance, double dt) const;
};

class ConstantVelocityModel final : public MotionModel
{
public:
	explicit ConstantVelocityModel(MotionModelConfig config = {});

	std::string name() const override;
	State transition(const State & state, double dt) const override;
	Covariance jacobian(const State & state, double dt) const override;
	Covariance processNoise(double dt) const override;

private:
	MotionModelConfig config_;
};

class ConstantAccelerationModel final : public MotionModel
{
public:
	explicit ConstantAccelerationModel(MotionModelConfig config = {});

	std::string name() const override;
	State transition(const State & state, double dt) const override;
	Covariance jacobian(const State & state, double dt) const override;
	Covariance processNoise(double dt) const override;

private:
	MotionModelConfig config_;
};

class SingerModel final : public MotionModel
{
public:
	explicit SingerModel(MotionModelConfig config = {});

	std::string name() const override;
	State transition(const State & state, double dt) const override;
	Covariance jacobian(const State & state, double dt) const override;
	Covariance processNoise(double dt) const override;

private:
	MotionModelConfig config_;
};

class ConstantTurnModel final : public MotionModel
{
public:
	explicit ConstantTurnModel(MotionModelConfig config = {});

	std::string name() const override;
	State transition(const State & state, double dt) const override;
	Covariance jacobian(const State & state, double dt) const override;
	Covariance processNoise(double dt) const override;

private:
	MotionModelConfig config_;
};

std::unique_ptr<MotionModel> createMotionModel(
	MotionModelType type,
	const MotionModelConfig & config = {});

class ImmkfPredictor
{
public:
	explicit ImmkfPredictor(const ImmkfConfig & config = {});

	void reset(const State & state, const Covariance & covariance);
	void predict(double dt);
	void update(const Measurement & measurement);
	std::vector<State> predictTrajectory(double dt, std::size_t horizon) const;

	const State & fusedState() const;
	const Covariance & fusedCovariance() const;
	std::array<double, 4> modeProbabilities() const;
	bool isInitialized() const;

	static MeasurementMatrix defaultMeasurementMatrix();
	static TransitionMatrix defaultTransitionMatrix();

private:
	static constexpr std::size_t kModelCount = 4;

	ImmkfConfig config_;
	std::array<std::unique_ptr<MotionModel>, kModelCount> models_{};
	std::array<State, kModelCount> model_states_{};
	std::array<Covariance, kModelCount> model_covariances_{};
	std::array<double, kModelCount> mode_probabilities_{};
	State fused_state_ = State::Zero();
	Covariance fused_covariance_ = Covariance::Identity();
	bool initialized_ = false;

	void rebuildFusedEstimate();
	void mixModelStates();
	double measurementLikelihood(std::size_t index, const Measurement & measurement) const;
};

struct TrackState
{
	std::string track_id;
	std::shared_ptr<ImmkfPredictor> predictor;
	std::vector<State> trajectory;
	double last_update_time = 0.0;
};

class TrackManager
{
public:
	explicit TrackManager(const ImmkfConfig & config = {});

	// Get or create a track predictor for the given track_id
	std::shared_ptr<ImmkfPredictor> getOrCreateTrack(const std::string & track_id);

	// Predict all tracks
	void predictAll(double dt);

	// Update a specific track with measurement
	void updateTrack(const std::string & track_id, const Measurement & measurement);

	// Get trajectory for a specific track
	std::vector<State> getTrajectory(const std::string & track_id, double dt, std::size_t horizon) const;

	// Get all track states
	std::vector<TrackState> getAllTracks() const;

	// Remove inactive tracks (older than timeout_seconds)
	void pruneInactiveTracks(double current_time, double timeout_seconds);

	// Clear all tracks
	void clear();

	// Get number of active tracks
	std::size_t size() const;

private:
	ImmkfConfig config_;
	std::unordered_map<std::string, std::shared_ptr<ImmkfPredictor>> tracks_;
	std::unordered_map<std::string, double> last_update_times_;
};

}  // namespace immkf_predictor

#endif  // IMMKF_PREDICTOR_IMMKF_HPP
