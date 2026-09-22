builtin_interfaces/Time stamp
uint64 session
uint64 last_sequence
# HOLDING, TRACKING, STOPPING, STOPPED. Stop is latched until lifecycle reactivation.
string state
string reason
# Measured joint state, never the requested target.
float64[7] positions
float64[7] velocities
