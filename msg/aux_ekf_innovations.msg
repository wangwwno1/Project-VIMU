uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

# Gyroscope
float32[12] gyro	# horizontal GPS velocity innovation (m/sec) and innovation variance ((m/sec)**2)

# TOPICS aux_ekf_innovations aux_ekf_innovation_variances
