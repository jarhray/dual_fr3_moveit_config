# stamp uses the controller's ROS clock; validity is also enforced on monotonic receipt time.
builtin_interfaces/Time stamp
uint64 sequence
uint64 session
float64 valid_for_s
float64[7] positions
float64[7] velocities
