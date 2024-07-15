# Resilient State Estimate
# Provides resilient estimates for vehicle state, including ground truth and estimated values.

uint64 timestamp        # time since system start (microseconds)

float32[4] rotor_speed  # Rotor speeds in RPM
float32[3] torque_ground_truth  # Ground truth torque in Nm
float32 thrust_ground_truth     # Ground truth thrust in N

float32[3] raw_angular_vel  # Raw IMU angular velocity in rad/s
float32[3] raw_acc          # Raw IMU acceleration in m/s^2

float32[3] imu_angular_vel  # IMU angular velocity in rad/s
float32[3] imu_acc          # IMU acceleration in m/s^2

float32[4] attitude_standard  # Standard attitude quaternion (w, x, y, z)

float32[3] torque_estimate  # Estimated torque in Nm
float32 thrust_estimate     # Estimated thrust in N

float32[4] attitude_estimate  # Estimated attitude quaternion (w, x, y, z)
float32[3] angular_vel_estimate  # Estimated angular velocity in rad/s

float32 anomaly_score         # Anomaly score
float32 anomaly_detection_rate # Rate of anomaly detection in Hz
bool anomaly_flag             # Flag indicating if an anomaly was detected (true = detected)

float32[3] imu_acc_offset
float32[3] imu_gyro_offset
bool attack_flag
