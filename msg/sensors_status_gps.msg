#
# Sensor check metrics for gps
#
uint64 timestamp # time since system start (microseconds)

float32 test_ratio                # Max value of test ratios
float32 pos_test_ratio
float32 vel_test_ratio
bool healthy                    # sensor healthy
bool ref_gps_enabled
