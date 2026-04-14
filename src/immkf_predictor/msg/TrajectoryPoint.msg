# Single predicted trajectory point with state information
builtin_interfaces/Time timestamp
float64 x
float64 y
float64 z
float64 vx
float64 vy
float64 vz
float64 confidence           # Prediction confidence (0-1)
uint8 model_index           # Which model: 0=CV, 1=CA, 2=Singer
