# ResilientStateEstimate
# Author: HM - 05162024

std_msgs/Header header

# Rotor speeds, torque and thrusts
float32[4] rotor_speed
float32[3] torque_ground_truth
float32 thrust_ground_truth

# Raw IMU measurements
float32[3] raw_angular_vel
float32[3] raw_acc

# IMU measurements
float32[3] imu_angular_vel
float32[3] imu_acc

# Standard estimates
geometry_msgs/Quaternion attitude_standard

# Resilient estimates
float32[3] torque_estimate
float32 thrust_estimate
geometry_msgs/Quaternion attitude_estimate
float32[3] angular_vel_estimate

# Anomaly detection
float32 anomaly_score
float32 anomaly_detection_rate
bool anomaly_flag

# Attack information
float32[3] imu_acc_offset
float32[3] imu_gyro_offset
bool attack_flag

