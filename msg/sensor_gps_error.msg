#
# Sensor check metrics for gps
#
uint64 timestamp # time since system start (microseconds)

float32[3] position_error       # Difference between sensor and reference in position (m)
float32[3] velocity_error       # Difference between sensor and reference in velocity (m/s)

# TOPICS sensors_gps_error sensors_gps_error_variances sensors_gps_error_test_ratios
