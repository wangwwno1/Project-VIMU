#
# Sensor check metrics for gps
#
uint64 timestamp # time since system start (microseconds)
uint64 timestamp_reference

float32[3] position_error       # Difference between sensor and reference in position (m)
float32[3] velocity_error       # Difference between sensor and reference in velocity (m/s)

# TOPICS sensor_gps_error sensor_gps_error_variances sensor_gps_error_ratios
