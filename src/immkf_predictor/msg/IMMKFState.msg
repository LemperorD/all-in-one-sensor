# Debug message: internal IMMKF filter state
string object_id
uint32 track_age
float64 last_measurement_time
uint8 model_index             # Most likely model
float64[9] state_vector       # Full 9D state (zero-padded)
float64[81] covariance_diag   # Diagonal of P matrix (9x9)
