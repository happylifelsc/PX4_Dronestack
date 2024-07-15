# Robust Attitude msg
uint64 timestamp        # time since system start (microseconds)
float32[4] orientation # Quaternion components (w, x, y, z)
float32[3] body_rate # Angular velocity in the body frame
float32 thrust # Thrust
