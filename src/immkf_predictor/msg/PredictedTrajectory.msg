# Complete predicted trajectory from IMMKF for a single tracked object
std_msgs/Header header
string object_id             # Detection ID for tracking
float64 publish_timestamp
immkf_predictor/TrajectoryPoint[] trajectory  # Array of predicted points
float64[] model_probabilities  # IMM model mixing weights [3]
float64 track_confidence     # Track reliability metric (0-1)
