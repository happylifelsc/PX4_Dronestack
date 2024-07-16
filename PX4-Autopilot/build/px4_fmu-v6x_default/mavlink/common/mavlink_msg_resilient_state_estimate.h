#pragma once
// MESSAGE RESILIENT_STATE_ESTIMATE PACKING

#define MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE 237


typedef struct __mavlink_resilient_state_estimate_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float rotor_speed[4]; /*< [rpm] Rotor speeds.*/
 float torque_ground_truth[3]; /*< [Nm] Ground truth torque.*/
 float thrust_ground_truth; /*< [N] Ground truth thrust.*/
 float raw_angular_vel[3]; /*< [rad/s] Raw IMU angular velocity.*/
 float raw_acc[3]; /*< [m/s/s] Raw IMU acceleration.*/
 float imu_angular_vel[3]; /*< [rad/s] IMU angular velocity.*/
 float imu_acc[3]; /*< [m/s/s] IMU acceleration.*/
 float attitude_standard[4]; /*<  Standard attitude quaternion (w, x, y, z).*/
 float torque_estimate[3]; /*< [Nm] Estimated torque.*/
 float thrust_estimate; /*< [N] Estimated thrust.*/
 float attitude_estimate[4]; /*<  Estimated attitude quaternion (w, x, y, z).*/
 float angular_vel_estimate[3]; /*< [rad/s] Estimated angular velocity.*/
 float anomaly_score; /*<  Anomaly score.*/
 float anomaly_detection_rate; /*< [Hz] Rate of anomaly detection.*/
 float imu_acc_offset[3]; /*< [m/s^2] IMU acceleration offsets.*/
 float imu_gyro_offset[3]; /*< [rad/s] IMU gyroscope offsets*/
 uint8_t anomaly_flag; /*<  Flag indicating if an anomaly was detected (1 = true, 0 = false).*/
 uint8_t attack_flag; /*<  Flag indicating if an attack was initiated (1 = true, 0 = false).*/
} mavlink_resilient_state_estimate_t;

#define MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN 178
#define MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_MIN_LEN 178
#define MAVLINK_MSG_ID_237_LEN 178
#define MAVLINK_MSG_ID_237_MIN_LEN 178

#define MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_CRC 92
#define MAVLINK_MSG_ID_237_CRC 92

#define MAVLINK_MSG_RESILIENT_STATE_ESTIMATE_FIELD_ROTOR_SPEED_LEN 4
#define MAVLINK_MSG_RESILIENT_STATE_ESTIMATE_FIELD_TORQUE_GROUND_TRUTH_LEN 3
#define MAVLINK_MSG_RESILIENT_STATE_ESTIMATE_FIELD_RAW_ANGULAR_VEL_LEN 3
#define MAVLINK_MSG_RESILIENT_STATE_ESTIMATE_FIELD_RAW_ACC_LEN 3
#define MAVLINK_MSG_RESILIENT_STATE_ESTIMATE_FIELD_IMU_ANGULAR_VEL_LEN 3
#define MAVLINK_MSG_RESILIENT_STATE_ESTIMATE_FIELD_IMU_ACC_LEN 3
#define MAVLINK_MSG_RESILIENT_STATE_ESTIMATE_FIELD_ATTITUDE_STANDARD_LEN 4
#define MAVLINK_MSG_RESILIENT_STATE_ESTIMATE_FIELD_TORQUE_ESTIMATE_LEN 3
#define MAVLINK_MSG_RESILIENT_STATE_ESTIMATE_FIELD_ATTITUDE_ESTIMATE_LEN 4
#define MAVLINK_MSG_RESILIENT_STATE_ESTIMATE_FIELD_ANGULAR_VEL_ESTIMATE_LEN 3
#define MAVLINK_MSG_RESILIENT_STATE_ESTIMATE_FIELD_IMU_ACC_OFFSET_LEN 3
#define MAVLINK_MSG_RESILIENT_STATE_ESTIMATE_FIELD_IMU_GYRO_OFFSET_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RESILIENT_STATE_ESTIMATE { \
    237, \
    "RESILIENT_STATE_ESTIMATE", \
    19, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_resilient_state_estimate_t, time_boot_ms) }, \
         { "rotor_speed", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_resilient_state_estimate_t, rotor_speed) }, \
         { "torque_ground_truth", NULL, MAVLINK_TYPE_FLOAT, 3, 20, offsetof(mavlink_resilient_state_estimate_t, torque_ground_truth) }, \
         { "thrust_ground_truth", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_resilient_state_estimate_t, thrust_ground_truth) }, \
         { "raw_angular_vel", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_resilient_state_estimate_t, raw_angular_vel) }, \
         { "raw_acc", NULL, MAVLINK_TYPE_FLOAT, 3, 48, offsetof(mavlink_resilient_state_estimate_t, raw_acc) }, \
         { "imu_angular_vel", NULL, MAVLINK_TYPE_FLOAT, 3, 60, offsetof(mavlink_resilient_state_estimate_t, imu_angular_vel) }, \
         { "imu_acc", NULL, MAVLINK_TYPE_FLOAT, 3, 72, offsetof(mavlink_resilient_state_estimate_t, imu_acc) }, \
         { "attitude_standard", NULL, MAVLINK_TYPE_FLOAT, 4, 84, offsetof(mavlink_resilient_state_estimate_t, attitude_standard) }, \
         { "torque_estimate", NULL, MAVLINK_TYPE_FLOAT, 3, 100, offsetof(mavlink_resilient_state_estimate_t, torque_estimate) }, \
         { "thrust_estimate", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_resilient_state_estimate_t, thrust_estimate) }, \
         { "attitude_estimate", NULL, MAVLINK_TYPE_FLOAT, 4, 116, offsetof(mavlink_resilient_state_estimate_t, attitude_estimate) }, \
         { "angular_vel_estimate", NULL, MAVLINK_TYPE_FLOAT, 3, 132, offsetof(mavlink_resilient_state_estimate_t, angular_vel_estimate) }, \
         { "anomaly_score", NULL, MAVLINK_TYPE_FLOAT, 0, 144, offsetof(mavlink_resilient_state_estimate_t, anomaly_score) }, \
         { "anomaly_detection_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 148, offsetof(mavlink_resilient_state_estimate_t, anomaly_detection_rate) }, \
         { "anomaly_flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 176, offsetof(mavlink_resilient_state_estimate_t, anomaly_flag) }, \
         { "imu_acc_offset", NULL, MAVLINK_TYPE_FLOAT, 3, 152, offsetof(mavlink_resilient_state_estimate_t, imu_acc_offset) }, \
         { "imu_gyro_offset", NULL, MAVLINK_TYPE_FLOAT, 3, 164, offsetof(mavlink_resilient_state_estimate_t, imu_gyro_offset) }, \
         { "attack_flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 177, offsetof(mavlink_resilient_state_estimate_t, attack_flag) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RESILIENT_STATE_ESTIMATE { \
    "RESILIENT_STATE_ESTIMATE", \
    19, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_resilient_state_estimate_t, time_boot_ms) }, \
         { "rotor_speed", NULL, MAVLINK_TYPE_FLOAT, 4, 4, offsetof(mavlink_resilient_state_estimate_t, rotor_speed) }, \
         { "torque_ground_truth", NULL, MAVLINK_TYPE_FLOAT, 3, 20, offsetof(mavlink_resilient_state_estimate_t, torque_ground_truth) }, \
         { "thrust_ground_truth", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_resilient_state_estimate_t, thrust_ground_truth) }, \
         { "raw_angular_vel", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_resilient_state_estimate_t, raw_angular_vel) }, \
         { "raw_acc", NULL, MAVLINK_TYPE_FLOAT, 3, 48, offsetof(mavlink_resilient_state_estimate_t, raw_acc) }, \
         { "imu_angular_vel", NULL, MAVLINK_TYPE_FLOAT, 3, 60, offsetof(mavlink_resilient_state_estimate_t, imu_angular_vel) }, \
         { "imu_acc", NULL, MAVLINK_TYPE_FLOAT, 3, 72, offsetof(mavlink_resilient_state_estimate_t, imu_acc) }, \
         { "attitude_standard", NULL, MAVLINK_TYPE_FLOAT, 4, 84, offsetof(mavlink_resilient_state_estimate_t, attitude_standard) }, \
         { "torque_estimate", NULL, MAVLINK_TYPE_FLOAT, 3, 100, offsetof(mavlink_resilient_state_estimate_t, torque_estimate) }, \
         { "thrust_estimate", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_resilient_state_estimate_t, thrust_estimate) }, \
         { "attitude_estimate", NULL, MAVLINK_TYPE_FLOAT, 4, 116, offsetof(mavlink_resilient_state_estimate_t, attitude_estimate) }, \
         { "angular_vel_estimate", NULL, MAVLINK_TYPE_FLOAT, 3, 132, offsetof(mavlink_resilient_state_estimate_t, angular_vel_estimate) }, \
         { "anomaly_score", NULL, MAVLINK_TYPE_FLOAT, 0, 144, offsetof(mavlink_resilient_state_estimate_t, anomaly_score) }, \
         { "anomaly_detection_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 148, offsetof(mavlink_resilient_state_estimate_t, anomaly_detection_rate) }, \
         { "anomaly_flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 176, offsetof(mavlink_resilient_state_estimate_t, anomaly_flag) }, \
         { "imu_acc_offset", NULL, MAVLINK_TYPE_FLOAT, 3, 152, offsetof(mavlink_resilient_state_estimate_t, imu_acc_offset) }, \
         { "imu_gyro_offset", NULL, MAVLINK_TYPE_FLOAT, 3, 164, offsetof(mavlink_resilient_state_estimate_t, imu_gyro_offset) }, \
         { "attack_flag", NULL, MAVLINK_TYPE_UINT8_T, 0, 177, offsetof(mavlink_resilient_state_estimate_t, attack_flag) }, \
         } \
}
#endif

/**
 * @brief Pack a resilient_state_estimate message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param rotor_speed [rpm] Rotor speeds.
 * @param torque_ground_truth [Nm] Ground truth torque.
 * @param thrust_ground_truth [N] Ground truth thrust.
 * @param raw_angular_vel [rad/s] Raw IMU angular velocity.
 * @param raw_acc [m/s/s] Raw IMU acceleration.
 * @param imu_angular_vel [rad/s] IMU angular velocity.
 * @param imu_acc [m/s/s] IMU acceleration.
 * @param attitude_standard  Standard attitude quaternion (w, x, y, z).
 * @param torque_estimate [Nm] Estimated torque.
 * @param thrust_estimate [N] Estimated thrust.
 * @param attitude_estimate  Estimated attitude quaternion (w, x, y, z).
 * @param angular_vel_estimate [rad/s] Estimated angular velocity.
 * @param anomaly_score  Anomaly score.
 * @param anomaly_detection_rate [Hz] Rate of anomaly detection.
 * @param anomaly_flag  Flag indicating if an anomaly was detected (1 = true, 0 = false).
 * @param imu_acc_offset [m/s^2] IMU acceleration offsets.
 * @param imu_gyro_offset [rad/s] IMU gyroscope offsets
 * @param attack_flag  Flag indicating if an attack was initiated (1 = true, 0 = false).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_resilient_state_estimate_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, const float *rotor_speed, const float *torque_ground_truth, float thrust_ground_truth, const float *raw_angular_vel, const float *raw_acc, const float *imu_angular_vel, const float *imu_acc, const float *attitude_standard, const float *torque_estimate, float thrust_estimate, const float *attitude_estimate, const float *angular_vel_estimate, float anomaly_score, float anomaly_detection_rate, uint8_t anomaly_flag, const float *imu_acc_offset, const float *imu_gyro_offset, uint8_t attack_flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 32, thrust_ground_truth);
    _mav_put_float(buf, 112, thrust_estimate);
    _mav_put_float(buf, 144, anomaly_score);
    _mav_put_float(buf, 148, anomaly_detection_rate);
    _mav_put_uint8_t(buf, 176, anomaly_flag);
    _mav_put_uint8_t(buf, 177, attack_flag);
    _mav_put_float_array(buf, 4, rotor_speed, 4);
    _mav_put_float_array(buf, 20, torque_ground_truth, 3);
    _mav_put_float_array(buf, 36, raw_angular_vel, 3);
    _mav_put_float_array(buf, 48, raw_acc, 3);
    _mav_put_float_array(buf, 60, imu_angular_vel, 3);
    _mav_put_float_array(buf, 72, imu_acc, 3);
    _mav_put_float_array(buf, 84, attitude_standard, 4);
    _mav_put_float_array(buf, 100, torque_estimate, 3);
    _mav_put_float_array(buf, 116, attitude_estimate, 4);
    _mav_put_float_array(buf, 132, angular_vel_estimate, 3);
    _mav_put_float_array(buf, 152, imu_acc_offset, 3);
    _mav_put_float_array(buf, 164, imu_gyro_offset, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN);
#else
    mavlink_resilient_state_estimate_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.thrust_ground_truth = thrust_ground_truth;
    packet.thrust_estimate = thrust_estimate;
    packet.anomaly_score = anomaly_score;
    packet.anomaly_detection_rate = anomaly_detection_rate;
    packet.anomaly_flag = anomaly_flag;
    packet.attack_flag = attack_flag;
    mav_array_memcpy(packet.rotor_speed, rotor_speed, sizeof(float)*4);
    mav_array_memcpy(packet.torque_ground_truth, torque_ground_truth, sizeof(float)*3);
    mav_array_memcpy(packet.raw_angular_vel, raw_angular_vel, sizeof(float)*3);
    mav_array_memcpy(packet.raw_acc, raw_acc, sizeof(float)*3);
    mav_array_memcpy(packet.imu_angular_vel, imu_angular_vel, sizeof(float)*3);
    mav_array_memcpy(packet.imu_acc, imu_acc, sizeof(float)*3);
    mav_array_memcpy(packet.attitude_standard, attitude_standard, sizeof(float)*4);
    mav_array_memcpy(packet.torque_estimate, torque_estimate, sizeof(float)*3);
    mav_array_memcpy(packet.attitude_estimate, attitude_estimate, sizeof(float)*4);
    mav_array_memcpy(packet.angular_vel_estimate, angular_vel_estimate, sizeof(float)*3);
    mav_array_memcpy(packet.imu_acc_offset, imu_acc_offset, sizeof(float)*3);
    mav_array_memcpy(packet.imu_gyro_offset, imu_gyro_offset, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_CRC);
}

/**
 * @brief Pack a resilient_state_estimate message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param rotor_speed [rpm] Rotor speeds.
 * @param torque_ground_truth [Nm] Ground truth torque.
 * @param thrust_ground_truth [N] Ground truth thrust.
 * @param raw_angular_vel [rad/s] Raw IMU angular velocity.
 * @param raw_acc [m/s/s] Raw IMU acceleration.
 * @param imu_angular_vel [rad/s] IMU angular velocity.
 * @param imu_acc [m/s/s] IMU acceleration.
 * @param attitude_standard  Standard attitude quaternion (w, x, y, z).
 * @param torque_estimate [Nm] Estimated torque.
 * @param thrust_estimate [N] Estimated thrust.
 * @param attitude_estimate  Estimated attitude quaternion (w, x, y, z).
 * @param angular_vel_estimate [rad/s] Estimated angular velocity.
 * @param anomaly_score  Anomaly score.
 * @param anomaly_detection_rate [Hz] Rate of anomaly detection.
 * @param anomaly_flag  Flag indicating if an anomaly was detected (1 = true, 0 = false).
 * @param imu_acc_offset [m/s^2] IMU acceleration offsets.
 * @param imu_gyro_offset [rad/s] IMU gyroscope offsets
 * @param attack_flag  Flag indicating if an attack was initiated (1 = true, 0 = false).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_resilient_state_estimate_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,const float *rotor_speed,const float *torque_ground_truth,float thrust_ground_truth,const float *raw_angular_vel,const float *raw_acc,const float *imu_angular_vel,const float *imu_acc,const float *attitude_standard,const float *torque_estimate,float thrust_estimate,const float *attitude_estimate,const float *angular_vel_estimate,float anomaly_score,float anomaly_detection_rate,uint8_t anomaly_flag,const float *imu_acc_offset,const float *imu_gyro_offset,uint8_t attack_flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 32, thrust_ground_truth);
    _mav_put_float(buf, 112, thrust_estimate);
    _mav_put_float(buf, 144, anomaly_score);
    _mav_put_float(buf, 148, anomaly_detection_rate);
    _mav_put_uint8_t(buf, 176, anomaly_flag);
    _mav_put_uint8_t(buf, 177, attack_flag);
    _mav_put_float_array(buf, 4, rotor_speed, 4);
    _mav_put_float_array(buf, 20, torque_ground_truth, 3);
    _mav_put_float_array(buf, 36, raw_angular_vel, 3);
    _mav_put_float_array(buf, 48, raw_acc, 3);
    _mav_put_float_array(buf, 60, imu_angular_vel, 3);
    _mav_put_float_array(buf, 72, imu_acc, 3);
    _mav_put_float_array(buf, 84, attitude_standard, 4);
    _mav_put_float_array(buf, 100, torque_estimate, 3);
    _mav_put_float_array(buf, 116, attitude_estimate, 4);
    _mav_put_float_array(buf, 132, angular_vel_estimate, 3);
    _mav_put_float_array(buf, 152, imu_acc_offset, 3);
    _mav_put_float_array(buf, 164, imu_gyro_offset, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN);
#else
    mavlink_resilient_state_estimate_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.thrust_ground_truth = thrust_ground_truth;
    packet.thrust_estimate = thrust_estimate;
    packet.anomaly_score = anomaly_score;
    packet.anomaly_detection_rate = anomaly_detection_rate;
    packet.anomaly_flag = anomaly_flag;
    packet.attack_flag = attack_flag;
    mav_array_memcpy(packet.rotor_speed, rotor_speed, sizeof(float)*4);
    mav_array_memcpy(packet.torque_ground_truth, torque_ground_truth, sizeof(float)*3);
    mav_array_memcpy(packet.raw_angular_vel, raw_angular_vel, sizeof(float)*3);
    mav_array_memcpy(packet.raw_acc, raw_acc, sizeof(float)*3);
    mav_array_memcpy(packet.imu_angular_vel, imu_angular_vel, sizeof(float)*3);
    mav_array_memcpy(packet.imu_acc, imu_acc, sizeof(float)*3);
    mav_array_memcpy(packet.attitude_standard, attitude_standard, sizeof(float)*4);
    mav_array_memcpy(packet.torque_estimate, torque_estimate, sizeof(float)*3);
    mav_array_memcpy(packet.attitude_estimate, attitude_estimate, sizeof(float)*4);
    mav_array_memcpy(packet.angular_vel_estimate, angular_vel_estimate, sizeof(float)*3);
    mav_array_memcpy(packet.imu_acc_offset, imu_acc_offset, sizeof(float)*3);
    mav_array_memcpy(packet.imu_gyro_offset, imu_gyro_offset, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_CRC);
}

/**
 * @brief Encode a resilient_state_estimate struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param resilient_state_estimate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_resilient_state_estimate_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_resilient_state_estimate_t* resilient_state_estimate)
{
    return mavlink_msg_resilient_state_estimate_pack(system_id, component_id, msg, resilient_state_estimate->time_boot_ms, resilient_state_estimate->rotor_speed, resilient_state_estimate->torque_ground_truth, resilient_state_estimate->thrust_ground_truth, resilient_state_estimate->raw_angular_vel, resilient_state_estimate->raw_acc, resilient_state_estimate->imu_angular_vel, resilient_state_estimate->imu_acc, resilient_state_estimate->attitude_standard, resilient_state_estimate->torque_estimate, resilient_state_estimate->thrust_estimate, resilient_state_estimate->attitude_estimate, resilient_state_estimate->angular_vel_estimate, resilient_state_estimate->anomaly_score, resilient_state_estimate->anomaly_detection_rate, resilient_state_estimate->anomaly_flag, resilient_state_estimate->imu_acc_offset, resilient_state_estimate->imu_gyro_offset, resilient_state_estimate->attack_flag);
}

/**
 * @brief Encode a resilient_state_estimate struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param resilient_state_estimate C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_resilient_state_estimate_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_resilient_state_estimate_t* resilient_state_estimate)
{
    return mavlink_msg_resilient_state_estimate_pack_chan(system_id, component_id, chan, msg, resilient_state_estimate->time_boot_ms, resilient_state_estimate->rotor_speed, resilient_state_estimate->torque_ground_truth, resilient_state_estimate->thrust_ground_truth, resilient_state_estimate->raw_angular_vel, resilient_state_estimate->raw_acc, resilient_state_estimate->imu_angular_vel, resilient_state_estimate->imu_acc, resilient_state_estimate->attitude_standard, resilient_state_estimate->torque_estimate, resilient_state_estimate->thrust_estimate, resilient_state_estimate->attitude_estimate, resilient_state_estimate->angular_vel_estimate, resilient_state_estimate->anomaly_score, resilient_state_estimate->anomaly_detection_rate, resilient_state_estimate->anomaly_flag, resilient_state_estimate->imu_acc_offset, resilient_state_estimate->imu_gyro_offset, resilient_state_estimate->attack_flag);
}

/**
 * @brief Send a resilient_state_estimate message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param rotor_speed [rpm] Rotor speeds.
 * @param torque_ground_truth [Nm] Ground truth torque.
 * @param thrust_ground_truth [N] Ground truth thrust.
 * @param raw_angular_vel [rad/s] Raw IMU angular velocity.
 * @param raw_acc [m/s/s] Raw IMU acceleration.
 * @param imu_angular_vel [rad/s] IMU angular velocity.
 * @param imu_acc [m/s/s] IMU acceleration.
 * @param attitude_standard  Standard attitude quaternion (w, x, y, z).
 * @param torque_estimate [Nm] Estimated torque.
 * @param thrust_estimate [N] Estimated thrust.
 * @param attitude_estimate  Estimated attitude quaternion (w, x, y, z).
 * @param angular_vel_estimate [rad/s] Estimated angular velocity.
 * @param anomaly_score  Anomaly score.
 * @param anomaly_detection_rate [Hz] Rate of anomaly detection.
 * @param anomaly_flag  Flag indicating if an anomaly was detected (1 = true, 0 = false).
 * @param imu_acc_offset [m/s^2] IMU acceleration offsets.
 * @param imu_gyro_offset [rad/s] IMU gyroscope offsets
 * @param attack_flag  Flag indicating if an attack was initiated (1 = true, 0 = false).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_resilient_state_estimate_send(mavlink_channel_t chan, uint32_t time_boot_ms, const float *rotor_speed, const float *torque_ground_truth, float thrust_ground_truth, const float *raw_angular_vel, const float *raw_acc, const float *imu_angular_vel, const float *imu_acc, const float *attitude_standard, const float *torque_estimate, float thrust_estimate, const float *attitude_estimate, const float *angular_vel_estimate, float anomaly_score, float anomaly_detection_rate, uint8_t anomaly_flag, const float *imu_acc_offset, const float *imu_gyro_offset, uint8_t attack_flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 32, thrust_ground_truth);
    _mav_put_float(buf, 112, thrust_estimate);
    _mav_put_float(buf, 144, anomaly_score);
    _mav_put_float(buf, 148, anomaly_detection_rate);
    _mav_put_uint8_t(buf, 176, anomaly_flag);
    _mav_put_uint8_t(buf, 177, attack_flag);
    _mav_put_float_array(buf, 4, rotor_speed, 4);
    _mav_put_float_array(buf, 20, torque_ground_truth, 3);
    _mav_put_float_array(buf, 36, raw_angular_vel, 3);
    _mav_put_float_array(buf, 48, raw_acc, 3);
    _mav_put_float_array(buf, 60, imu_angular_vel, 3);
    _mav_put_float_array(buf, 72, imu_acc, 3);
    _mav_put_float_array(buf, 84, attitude_standard, 4);
    _mav_put_float_array(buf, 100, torque_estimate, 3);
    _mav_put_float_array(buf, 116, attitude_estimate, 4);
    _mav_put_float_array(buf, 132, angular_vel_estimate, 3);
    _mav_put_float_array(buf, 152, imu_acc_offset, 3);
    _mav_put_float_array(buf, 164, imu_gyro_offset, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE, buf, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_CRC);
#else
    mavlink_resilient_state_estimate_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.thrust_ground_truth = thrust_ground_truth;
    packet.thrust_estimate = thrust_estimate;
    packet.anomaly_score = anomaly_score;
    packet.anomaly_detection_rate = anomaly_detection_rate;
    packet.anomaly_flag = anomaly_flag;
    packet.attack_flag = attack_flag;
    mav_array_memcpy(packet.rotor_speed, rotor_speed, sizeof(float)*4);
    mav_array_memcpy(packet.torque_ground_truth, torque_ground_truth, sizeof(float)*3);
    mav_array_memcpy(packet.raw_angular_vel, raw_angular_vel, sizeof(float)*3);
    mav_array_memcpy(packet.raw_acc, raw_acc, sizeof(float)*3);
    mav_array_memcpy(packet.imu_angular_vel, imu_angular_vel, sizeof(float)*3);
    mav_array_memcpy(packet.imu_acc, imu_acc, sizeof(float)*3);
    mav_array_memcpy(packet.attitude_standard, attitude_standard, sizeof(float)*4);
    mav_array_memcpy(packet.torque_estimate, torque_estimate, sizeof(float)*3);
    mav_array_memcpy(packet.attitude_estimate, attitude_estimate, sizeof(float)*4);
    mav_array_memcpy(packet.angular_vel_estimate, angular_vel_estimate, sizeof(float)*3);
    mav_array_memcpy(packet.imu_acc_offset, imu_acc_offset, sizeof(float)*3);
    mav_array_memcpy(packet.imu_gyro_offset, imu_gyro_offset, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE, (const char *)&packet, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_CRC);
#endif
}

/**
 * @brief Send a resilient_state_estimate message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_resilient_state_estimate_send_struct(mavlink_channel_t chan, const mavlink_resilient_state_estimate_t* resilient_state_estimate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_resilient_state_estimate_send(chan, resilient_state_estimate->time_boot_ms, resilient_state_estimate->rotor_speed, resilient_state_estimate->torque_ground_truth, resilient_state_estimate->thrust_ground_truth, resilient_state_estimate->raw_angular_vel, resilient_state_estimate->raw_acc, resilient_state_estimate->imu_angular_vel, resilient_state_estimate->imu_acc, resilient_state_estimate->attitude_standard, resilient_state_estimate->torque_estimate, resilient_state_estimate->thrust_estimate, resilient_state_estimate->attitude_estimate, resilient_state_estimate->angular_vel_estimate, resilient_state_estimate->anomaly_score, resilient_state_estimate->anomaly_detection_rate, resilient_state_estimate->anomaly_flag, resilient_state_estimate->imu_acc_offset, resilient_state_estimate->imu_gyro_offset, resilient_state_estimate->attack_flag);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE, (const char *)resilient_state_estimate, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_resilient_state_estimate_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, const float *rotor_speed, const float *torque_ground_truth, float thrust_ground_truth, const float *raw_angular_vel, const float *raw_acc, const float *imu_angular_vel, const float *imu_acc, const float *attitude_standard, const float *torque_estimate, float thrust_estimate, const float *attitude_estimate, const float *angular_vel_estimate, float anomaly_score, float anomaly_detection_rate, uint8_t anomaly_flag, const float *imu_acc_offset, const float *imu_gyro_offset, uint8_t attack_flag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 32, thrust_ground_truth);
    _mav_put_float(buf, 112, thrust_estimate);
    _mav_put_float(buf, 144, anomaly_score);
    _mav_put_float(buf, 148, anomaly_detection_rate);
    _mav_put_uint8_t(buf, 176, anomaly_flag);
    _mav_put_uint8_t(buf, 177, attack_flag);
    _mav_put_float_array(buf, 4, rotor_speed, 4);
    _mav_put_float_array(buf, 20, torque_ground_truth, 3);
    _mav_put_float_array(buf, 36, raw_angular_vel, 3);
    _mav_put_float_array(buf, 48, raw_acc, 3);
    _mav_put_float_array(buf, 60, imu_angular_vel, 3);
    _mav_put_float_array(buf, 72, imu_acc, 3);
    _mav_put_float_array(buf, 84, attitude_standard, 4);
    _mav_put_float_array(buf, 100, torque_estimate, 3);
    _mav_put_float_array(buf, 116, attitude_estimate, 4);
    _mav_put_float_array(buf, 132, angular_vel_estimate, 3);
    _mav_put_float_array(buf, 152, imu_acc_offset, 3);
    _mav_put_float_array(buf, 164, imu_gyro_offset, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE, buf, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_CRC);
#else
    mavlink_resilient_state_estimate_t *packet = (mavlink_resilient_state_estimate_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->thrust_ground_truth = thrust_ground_truth;
    packet->thrust_estimate = thrust_estimate;
    packet->anomaly_score = anomaly_score;
    packet->anomaly_detection_rate = anomaly_detection_rate;
    packet->anomaly_flag = anomaly_flag;
    packet->attack_flag = attack_flag;
    mav_array_memcpy(packet->rotor_speed, rotor_speed, sizeof(float)*4);
    mav_array_memcpy(packet->torque_ground_truth, torque_ground_truth, sizeof(float)*3);
    mav_array_memcpy(packet->raw_angular_vel, raw_angular_vel, sizeof(float)*3);
    mav_array_memcpy(packet->raw_acc, raw_acc, sizeof(float)*3);
    mav_array_memcpy(packet->imu_angular_vel, imu_angular_vel, sizeof(float)*3);
    mav_array_memcpy(packet->imu_acc, imu_acc, sizeof(float)*3);
    mav_array_memcpy(packet->attitude_standard, attitude_standard, sizeof(float)*4);
    mav_array_memcpy(packet->torque_estimate, torque_estimate, sizeof(float)*3);
    mav_array_memcpy(packet->attitude_estimate, attitude_estimate, sizeof(float)*4);
    mav_array_memcpy(packet->angular_vel_estimate, angular_vel_estimate, sizeof(float)*3);
    mav_array_memcpy(packet->imu_acc_offset, imu_acc_offset, sizeof(float)*3);
    mav_array_memcpy(packet->imu_gyro_offset, imu_gyro_offset, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE, (const char *)packet, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_MIN_LEN, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_CRC);
#endif
}
#endif

#endif

// MESSAGE RESILIENT_STATE_ESTIMATE UNPACKING


/**
 * @brief Get field time_boot_ms from resilient_state_estimate message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_resilient_state_estimate_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field rotor_speed from resilient_state_estimate message
 *
 * @return [rpm] Rotor speeds.
 */
static inline uint16_t mavlink_msg_resilient_state_estimate_get_rotor_speed(const mavlink_message_t* msg, float *rotor_speed)
{
    return _MAV_RETURN_float_array(msg, rotor_speed, 4,  4);
}

/**
 * @brief Get field torque_ground_truth from resilient_state_estimate message
 *
 * @return [Nm] Ground truth torque.
 */
static inline uint16_t mavlink_msg_resilient_state_estimate_get_torque_ground_truth(const mavlink_message_t* msg, float *torque_ground_truth)
{
    return _MAV_RETURN_float_array(msg, torque_ground_truth, 3,  20);
}

/**
 * @brief Get field thrust_ground_truth from resilient_state_estimate message
 *
 * @return [N] Ground truth thrust.
 */
static inline float mavlink_msg_resilient_state_estimate_get_thrust_ground_truth(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field raw_angular_vel from resilient_state_estimate message
 *
 * @return [rad/s] Raw IMU angular velocity.
 */
static inline uint16_t mavlink_msg_resilient_state_estimate_get_raw_angular_vel(const mavlink_message_t* msg, float *raw_angular_vel)
{
    return _MAV_RETURN_float_array(msg, raw_angular_vel, 3,  36);
}

/**
 * @brief Get field raw_acc from resilient_state_estimate message
 *
 * @return [m/s/s] Raw IMU acceleration.
 */
static inline uint16_t mavlink_msg_resilient_state_estimate_get_raw_acc(const mavlink_message_t* msg, float *raw_acc)
{
    return _MAV_RETURN_float_array(msg, raw_acc, 3,  48);
}

/**
 * @brief Get field imu_angular_vel from resilient_state_estimate message
 *
 * @return [rad/s] IMU angular velocity.
 */
static inline uint16_t mavlink_msg_resilient_state_estimate_get_imu_angular_vel(const mavlink_message_t* msg, float *imu_angular_vel)
{
    return _MAV_RETURN_float_array(msg, imu_angular_vel, 3,  60);
}

/**
 * @brief Get field imu_acc from resilient_state_estimate message
 *
 * @return [m/s/s] IMU acceleration.
 */
static inline uint16_t mavlink_msg_resilient_state_estimate_get_imu_acc(const mavlink_message_t* msg, float *imu_acc)
{
    return _MAV_RETURN_float_array(msg, imu_acc, 3,  72);
}

/**
 * @brief Get field attitude_standard from resilient_state_estimate message
 *
 * @return  Standard attitude quaternion (w, x, y, z).
 */
static inline uint16_t mavlink_msg_resilient_state_estimate_get_attitude_standard(const mavlink_message_t* msg, float *attitude_standard)
{
    return _MAV_RETURN_float_array(msg, attitude_standard, 4,  84);
}

/**
 * @brief Get field torque_estimate from resilient_state_estimate message
 *
 * @return [Nm] Estimated torque.
 */
static inline uint16_t mavlink_msg_resilient_state_estimate_get_torque_estimate(const mavlink_message_t* msg, float *torque_estimate)
{
    return _MAV_RETURN_float_array(msg, torque_estimate, 3,  100);
}

/**
 * @brief Get field thrust_estimate from resilient_state_estimate message
 *
 * @return [N] Estimated thrust.
 */
static inline float mavlink_msg_resilient_state_estimate_get_thrust_estimate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  112);
}

/**
 * @brief Get field attitude_estimate from resilient_state_estimate message
 *
 * @return  Estimated attitude quaternion (w, x, y, z).
 */
static inline uint16_t mavlink_msg_resilient_state_estimate_get_attitude_estimate(const mavlink_message_t* msg, float *attitude_estimate)
{
    return _MAV_RETURN_float_array(msg, attitude_estimate, 4,  116);
}

/**
 * @brief Get field angular_vel_estimate from resilient_state_estimate message
 *
 * @return [rad/s] Estimated angular velocity.
 */
static inline uint16_t mavlink_msg_resilient_state_estimate_get_angular_vel_estimate(const mavlink_message_t* msg, float *angular_vel_estimate)
{
    return _MAV_RETURN_float_array(msg, angular_vel_estimate, 3,  132);
}

/**
 * @brief Get field anomaly_score from resilient_state_estimate message
 *
 * @return  Anomaly score.
 */
static inline float mavlink_msg_resilient_state_estimate_get_anomaly_score(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  144);
}

/**
 * @brief Get field anomaly_detection_rate from resilient_state_estimate message
 *
 * @return [Hz] Rate of anomaly detection.
 */
static inline float mavlink_msg_resilient_state_estimate_get_anomaly_detection_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  148);
}

/**
 * @brief Get field anomaly_flag from resilient_state_estimate message
 *
 * @return  Flag indicating if an anomaly was detected (1 = true, 0 = false).
 */
static inline uint8_t mavlink_msg_resilient_state_estimate_get_anomaly_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  176);
}

/**
 * @brief Get field imu_acc_offset from resilient_state_estimate message
 *
 * @return [m/s^2] IMU acceleration offsets.
 */
static inline uint16_t mavlink_msg_resilient_state_estimate_get_imu_acc_offset(const mavlink_message_t* msg, float *imu_acc_offset)
{
    return _MAV_RETURN_float_array(msg, imu_acc_offset, 3,  152);
}

/**
 * @brief Get field imu_gyro_offset from resilient_state_estimate message
 *
 * @return [rad/s] IMU gyroscope offsets
 */
static inline uint16_t mavlink_msg_resilient_state_estimate_get_imu_gyro_offset(const mavlink_message_t* msg, float *imu_gyro_offset)
{
    return _MAV_RETURN_float_array(msg, imu_gyro_offset, 3,  164);
}

/**
 * @brief Get field attack_flag from resilient_state_estimate message
 *
 * @return  Flag indicating if an attack was initiated (1 = true, 0 = false).
 */
static inline uint8_t mavlink_msg_resilient_state_estimate_get_attack_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  177);
}

/**
 * @brief Decode a resilient_state_estimate message into a struct
 *
 * @param msg The message to decode
 * @param resilient_state_estimate C-struct to decode the message contents into
 */
static inline void mavlink_msg_resilient_state_estimate_decode(const mavlink_message_t* msg, mavlink_resilient_state_estimate_t* resilient_state_estimate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    resilient_state_estimate->time_boot_ms = mavlink_msg_resilient_state_estimate_get_time_boot_ms(msg);
    mavlink_msg_resilient_state_estimate_get_rotor_speed(msg, resilient_state_estimate->rotor_speed);
    mavlink_msg_resilient_state_estimate_get_torque_ground_truth(msg, resilient_state_estimate->torque_ground_truth);
    resilient_state_estimate->thrust_ground_truth = mavlink_msg_resilient_state_estimate_get_thrust_ground_truth(msg);
    mavlink_msg_resilient_state_estimate_get_raw_angular_vel(msg, resilient_state_estimate->raw_angular_vel);
    mavlink_msg_resilient_state_estimate_get_raw_acc(msg, resilient_state_estimate->raw_acc);
    mavlink_msg_resilient_state_estimate_get_imu_angular_vel(msg, resilient_state_estimate->imu_angular_vel);
    mavlink_msg_resilient_state_estimate_get_imu_acc(msg, resilient_state_estimate->imu_acc);
    mavlink_msg_resilient_state_estimate_get_attitude_standard(msg, resilient_state_estimate->attitude_standard);
    mavlink_msg_resilient_state_estimate_get_torque_estimate(msg, resilient_state_estimate->torque_estimate);
    resilient_state_estimate->thrust_estimate = mavlink_msg_resilient_state_estimate_get_thrust_estimate(msg);
    mavlink_msg_resilient_state_estimate_get_attitude_estimate(msg, resilient_state_estimate->attitude_estimate);
    mavlink_msg_resilient_state_estimate_get_angular_vel_estimate(msg, resilient_state_estimate->angular_vel_estimate);
    resilient_state_estimate->anomaly_score = mavlink_msg_resilient_state_estimate_get_anomaly_score(msg);
    resilient_state_estimate->anomaly_detection_rate = mavlink_msg_resilient_state_estimate_get_anomaly_detection_rate(msg);
    mavlink_msg_resilient_state_estimate_get_imu_acc_offset(msg, resilient_state_estimate->imu_acc_offset);
    mavlink_msg_resilient_state_estimate_get_imu_gyro_offset(msg, resilient_state_estimate->imu_gyro_offset);
    resilient_state_estimate->anomaly_flag = mavlink_msg_resilient_state_estimate_get_anomaly_flag(msg);
    resilient_state_estimate->attack_flag = mavlink_msg_resilient_state_estimate_get_attack_flag(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN? msg->len : MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN;
        memset(resilient_state_estimate, 0, MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN);
    memcpy(resilient_state_estimate, _MAV_PAYLOAD(msg), len);
#endif
}
