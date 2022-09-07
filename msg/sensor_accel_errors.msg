uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_start    # time at the record start

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

uint8 samples             # number of valid samples

float32[16] x               # error ratio in the FRD board frame X-axis in m/s^2
float32[16] y               # error ratio in the FRD board frame Y-axis in m/s^2
float32[16] z               # error ratio in the FRD board frame Z-axis in m/s^2
float32[16] test_ratio      # test ratio for accel detector
