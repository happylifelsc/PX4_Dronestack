# IMU Attack Message
uint64 timestamp        # time since system start (microseconds)
float32 x_acc_offset    # Offset of acceleration in X direction
float32 y_acc_offset    # Offset of acceleration in Y direction
float32 z_acc_offset    # Offset of acceleration in Z direction
float32 x_gyro_offset   # Offset of gyroscope in X direction
float32 y_gyro_offset   # Offset of gyroscope in Y direction
float32 z_gyro_offset   # Offset of gyroscope in Z direction
bool attack_flag        # Attack flag, true if attack detected
