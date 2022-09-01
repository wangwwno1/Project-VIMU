uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32[3]  ang_rate_delayed_raw  # raw angular rates calculated from imu at delayed horizon
float32 dt_ekf_avg
float32 dt_imu_avg
float32 baro_hgt_offset

# TOPICS estimator_offset_states vehicle_offset_states
