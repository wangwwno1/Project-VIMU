uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the last update (microseconds)
uint64 timestamp_actuator_output         # the timestamp of the last actuator setpoint (microseconds)
uint64 timestamp_takeoff
uint64 timestamp_landed

bool at_rest
bool landed
bool in_air
bool publish
bool all_imu_compromised

float32[3] control_acceleration
float32[3] control_torque
float32[3] external_acceleration
float32[3] external_angular_acceleration
