#
# Sensor check metrics. This will be zero for a sensor that's primary or unpopulated.
#
uint64 timestamp # time since system start (microseconds)

uint32 device_id_primary       # current primary device id for reference

uint32[4]  device_ids
uint64[4]  timestamp_reference
float32[4] error               # absolute error between barometer measurement and reference (m)
float32[4] variance
float32[4] test_ratio
