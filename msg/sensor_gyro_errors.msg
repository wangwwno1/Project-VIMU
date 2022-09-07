uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_start    # time at the record start

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

uint8 samples             # number of valid samples

float32[16] x               # angular velocity in the FRD board frame X-axis in rad/s
float32[16] y               # angular velocity in the FRD board frame Y-axis in rad/s
float32[16] z               # angular velocity in the FRD board frame Z-axis in rad/s
float32[16] test_ratio      # test ratio for accel detector
