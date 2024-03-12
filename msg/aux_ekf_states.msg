uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32[6] states		# Internal filter states

float32[6] covariances	# Diagonal Elements of Covariance Matrix

# TOPICS aux_ekf_states
