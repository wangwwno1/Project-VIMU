#
# Sensor check metrics. This will be zero for a sensor that's primary or unpopulated.
#
uint64 timestamp # time since system start (microseconds)

float32     gps_test_ratio
float32     baro_hgt_test_ratio

float32[4]  mag_test_ratios
uint32[4]   mag_device_ids          # unique device ID for the sensor that does not change between power cycles
bool[4]     healthy_ekf_for_mag
uint64[4]   last_mag_faulty

# TOPICS sensor_health_status