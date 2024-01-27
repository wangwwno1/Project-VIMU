uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)

float32[3] rel_wind_body            # velocity in air about the FRD body frame XYZ-axis in m/s
float32[3] acceleration             # angular acceleration about the FRD body frame XYZ-axis in rad/s^2
float32[3] angular_acceleration     # acceleration about the FRD body frame XYZ-axis in m/s^2

