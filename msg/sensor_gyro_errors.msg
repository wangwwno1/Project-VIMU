uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample   # time at the sample received

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

uint8 samples             # number of raw samples

float32 x               # error ratio in the FRD board frame X-axis in rad/s
float32 y               # error ratio in the FRD board frame Y-axis in rad/s
float32 z               # error ratio in the FRD board frame Z-axis in rad/s
float32 test_ratio      # test ratio for accel detector
