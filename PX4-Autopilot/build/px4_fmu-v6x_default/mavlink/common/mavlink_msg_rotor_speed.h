#pragma once
// MESSAGE ROTOR_SPEED PACKING

#define MAVLINK_MSG_ID_ROTOR_SPEED 227


typedef struct __mavlink_rotor_speed_t {
 float w_0; /*< [rad/s] Rotor 0 speed*/
 float w_1; /*< [rad/s] Rotor 1 speed*/
 float w_2; /*< [rad/s] Rotor 2 speed*/
 float w_3; /*< [rad/s] Rotor 3 speed*/
 float tau_x; /*< [Nm] Torque around X-axis*/
 float tau_y; /*< [Nm] Torque around Y-axis*/
 float tau_z; /*< [Nm] Torque around Z-axis*/
 float thrust; /*< [N] Total thrust*/
} mavlink_rotor_speed_t;

#define MAVLINK_MSG_ID_ROTOR_SPEED_LEN 32
#define MAVLINK_MSG_ID_ROTOR_SPEED_MIN_LEN 32
#define MAVLINK_MSG_ID_227_LEN 32
#define MAVLINK_MSG_ID_227_MIN_LEN 32

#define MAVLINK_MSG_ID_ROTOR_SPEED_CRC 52
#define MAVLINK_MSG_ID_227_CRC 52



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROTOR_SPEED { \
    227, \
    "ROTOR_SPEED", \
    8, \
    {  { "w_0", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_rotor_speed_t, w_0) }, \
         { "w_1", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_rotor_speed_t, w_1) }, \
         { "w_2", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_rotor_speed_t, w_2) }, \
         { "w_3", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_rotor_speed_t, w_3) }, \
         { "tau_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_rotor_speed_t, tau_x) }, \
         { "tau_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_rotor_speed_t, tau_y) }, \
         { "tau_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_rotor_speed_t, tau_z) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_rotor_speed_t, thrust) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROTOR_SPEED { \
    "ROTOR_SPEED", \
    8, \
    {  { "w_0", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_rotor_speed_t, w_0) }, \
         { "w_1", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_rotor_speed_t, w_1) }, \
         { "w_2", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_rotor_speed_t, w_2) }, \
         { "w_3", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_rotor_speed_t, w_3) }, \
         { "tau_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_rotor_speed_t, tau_x) }, \
         { "tau_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_rotor_speed_t, tau_y) }, \
         { "tau_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_rotor_speed_t, tau_z) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_rotor_speed_t, thrust) }, \
         } \
}
#endif

/**
 * @brief Pack a rotor_speed message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param w_0 [rad/s] Rotor 0 speed
 * @param w_1 [rad/s] Rotor 1 speed
 * @param w_2 [rad/s] Rotor 2 speed
 * @param w_3 [rad/s] Rotor 3 speed
 * @param tau_x [Nm] Torque around X-axis
 * @param tau_y [Nm] Torque around Y-axis
 * @param tau_z [Nm] Torque around Z-axis
 * @param thrust [N] Total thrust
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rotor_speed_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float w_0, float w_1, float w_2, float w_3, float tau_x, float tau_y, float tau_z, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROTOR_SPEED_LEN];
    _mav_put_float(buf, 0, w_0);
    _mav_put_float(buf, 4, w_1);
    _mav_put_float(buf, 8, w_2);
    _mav_put_float(buf, 12, w_3);
    _mav_put_float(buf, 16, tau_x);
    _mav_put_float(buf, 20, tau_y);
    _mav_put_float(buf, 24, tau_z);
    _mav_put_float(buf, 28, thrust);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROTOR_SPEED_LEN);
#else
    mavlink_rotor_speed_t packet;
    packet.w_0 = w_0;
    packet.w_1 = w_1;
    packet.w_2 = w_2;
    packet.w_3 = w_3;
    packet.tau_x = tau_x;
    packet.tau_y = tau_y;
    packet.tau_z = tau_z;
    packet.thrust = thrust;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROTOR_SPEED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROTOR_SPEED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROTOR_SPEED_MIN_LEN, MAVLINK_MSG_ID_ROTOR_SPEED_LEN, MAVLINK_MSG_ID_ROTOR_SPEED_CRC);
}

/**
 * @brief Pack a rotor_speed message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param w_0 [rad/s] Rotor 0 speed
 * @param w_1 [rad/s] Rotor 1 speed
 * @param w_2 [rad/s] Rotor 2 speed
 * @param w_3 [rad/s] Rotor 3 speed
 * @param tau_x [Nm] Torque around X-axis
 * @param tau_y [Nm] Torque around Y-axis
 * @param tau_z [Nm] Torque around Z-axis
 * @param thrust [N] Total thrust
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rotor_speed_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float w_0,float w_1,float w_2,float w_3,float tau_x,float tau_y,float tau_z,float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROTOR_SPEED_LEN];
    _mav_put_float(buf, 0, w_0);
    _mav_put_float(buf, 4, w_1);
    _mav_put_float(buf, 8, w_2);
    _mav_put_float(buf, 12, w_3);
    _mav_put_float(buf, 16, tau_x);
    _mav_put_float(buf, 20, tau_y);
    _mav_put_float(buf, 24, tau_z);
    _mav_put_float(buf, 28, thrust);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROTOR_SPEED_LEN);
#else
    mavlink_rotor_speed_t packet;
    packet.w_0 = w_0;
    packet.w_1 = w_1;
    packet.w_2 = w_2;
    packet.w_3 = w_3;
    packet.tau_x = tau_x;
    packet.tau_y = tau_y;
    packet.tau_z = tau_z;
    packet.thrust = thrust;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROTOR_SPEED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROTOR_SPEED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROTOR_SPEED_MIN_LEN, MAVLINK_MSG_ID_ROTOR_SPEED_LEN, MAVLINK_MSG_ID_ROTOR_SPEED_CRC);
}

/**
 * @brief Encode a rotor_speed struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rotor_speed C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rotor_speed_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rotor_speed_t* rotor_speed)
{
    return mavlink_msg_rotor_speed_pack(system_id, component_id, msg, rotor_speed->w_0, rotor_speed->w_1, rotor_speed->w_2, rotor_speed->w_3, rotor_speed->tau_x, rotor_speed->tau_y, rotor_speed->tau_z, rotor_speed->thrust);
}

/**
 * @brief Encode a rotor_speed struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rotor_speed C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rotor_speed_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rotor_speed_t* rotor_speed)
{
    return mavlink_msg_rotor_speed_pack_chan(system_id, component_id, chan, msg, rotor_speed->w_0, rotor_speed->w_1, rotor_speed->w_2, rotor_speed->w_3, rotor_speed->tau_x, rotor_speed->tau_y, rotor_speed->tau_z, rotor_speed->thrust);
}

/**
 * @brief Send a rotor_speed message
 * @param chan MAVLink channel to send the message
 *
 * @param w_0 [rad/s] Rotor 0 speed
 * @param w_1 [rad/s] Rotor 1 speed
 * @param w_2 [rad/s] Rotor 2 speed
 * @param w_3 [rad/s] Rotor 3 speed
 * @param tau_x [Nm] Torque around X-axis
 * @param tau_y [Nm] Torque around Y-axis
 * @param tau_z [Nm] Torque around Z-axis
 * @param thrust [N] Total thrust
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rotor_speed_send(mavlink_channel_t chan, float w_0, float w_1, float w_2, float w_3, float tau_x, float tau_y, float tau_z, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROTOR_SPEED_LEN];
    _mav_put_float(buf, 0, w_0);
    _mav_put_float(buf, 4, w_1);
    _mav_put_float(buf, 8, w_2);
    _mav_put_float(buf, 12, w_3);
    _mav_put_float(buf, 16, tau_x);
    _mav_put_float(buf, 20, tau_y);
    _mav_put_float(buf, 24, tau_z);
    _mav_put_float(buf, 28, thrust);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROTOR_SPEED, buf, MAVLINK_MSG_ID_ROTOR_SPEED_MIN_LEN, MAVLINK_MSG_ID_ROTOR_SPEED_LEN, MAVLINK_MSG_ID_ROTOR_SPEED_CRC);
#else
    mavlink_rotor_speed_t packet;
    packet.w_0 = w_0;
    packet.w_1 = w_1;
    packet.w_2 = w_2;
    packet.w_3 = w_3;
    packet.tau_x = tau_x;
    packet.tau_y = tau_y;
    packet.tau_z = tau_z;
    packet.thrust = thrust;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROTOR_SPEED, (const char *)&packet, MAVLINK_MSG_ID_ROTOR_SPEED_MIN_LEN, MAVLINK_MSG_ID_ROTOR_SPEED_LEN, MAVLINK_MSG_ID_ROTOR_SPEED_CRC);
#endif
}

/**
 * @brief Send a rotor_speed message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rotor_speed_send_struct(mavlink_channel_t chan, const mavlink_rotor_speed_t* rotor_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rotor_speed_send(chan, rotor_speed->w_0, rotor_speed->w_1, rotor_speed->w_2, rotor_speed->w_3, rotor_speed->tau_x, rotor_speed->tau_y, rotor_speed->tau_z, rotor_speed->thrust);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROTOR_SPEED, (const char *)rotor_speed, MAVLINK_MSG_ID_ROTOR_SPEED_MIN_LEN, MAVLINK_MSG_ID_ROTOR_SPEED_LEN, MAVLINK_MSG_ID_ROTOR_SPEED_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROTOR_SPEED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rotor_speed_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float w_0, float w_1, float w_2, float w_3, float tau_x, float tau_y, float tau_z, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, w_0);
    _mav_put_float(buf, 4, w_1);
    _mav_put_float(buf, 8, w_2);
    _mav_put_float(buf, 12, w_3);
    _mav_put_float(buf, 16, tau_x);
    _mav_put_float(buf, 20, tau_y);
    _mav_put_float(buf, 24, tau_z);
    _mav_put_float(buf, 28, thrust);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROTOR_SPEED, buf, MAVLINK_MSG_ID_ROTOR_SPEED_MIN_LEN, MAVLINK_MSG_ID_ROTOR_SPEED_LEN, MAVLINK_MSG_ID_ROTOR_SPEED_CRC);
#else
    mavlink_rotor_speed_t *packet = (mavlink_rotor_speed_t *)msgbuf;
    packet->w_0 = w_0;
    packet->w_1 = w_1;
    packet->w_2 = w_2;
    packet->w_3 = w_3;
    packet->tau_x = tau_x;
    packet->tau_y = tau_y;
    packet->tau_z = tau_z;
    packet->thrust = thrust;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROTOR_SPEED, (const char *)packet, MAVLINK_MSG_ID_ROTOR_SPEED_MIN_LEN, MAVLINK_MSG_ID_ROTOR_SPEED_LEN, MAVLINK_MSG_ID_ROTOR_SPEED_CRC);
#endif
}
#endif

#endif

// MESSAGE ROTOR_SPEED UNPACKING


/**
 * @brief Get field w_0 from rotor_speed message
 *
 * @return [rad/s] Rotor 0 speed
 */
static inline float mavlink_msg_rotor_speed_get_w_0(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field w_1 from rotor_speed message
 *
 * @return [rad/s] Rotor 1 speed
 */
static inline float mavlink_msg_rotor_speed_get_w_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field w_2 from rotor_speed message
 *
 * @return [rad/s] Rotor 2 speed
 */
static inline float mavlink_msg_rotor_speed_get_w_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field w_3 from rotor_speed message
 *
 * @return [rad/s] Rotor 3 speed
 */
static inline float mavlink_msg_rotor_speed_get_w_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field tau_x from rotor_speed message
 *
 * @return [Nm] Torque around X-axis
 */
static inline float mavlink_msg_rotor_speed_get_tau_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field tau_y from rotor_speed message
 *
 * @return [Nm] Torque around Y-axis
 */
static inline float mavlink_msg_rotor_speed_get_tau_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field tau_z from rotor_speed message
 *
 * @return [Nm] Torque around Z-axis
 */
static inline float mavlink_msg_rotor_speed_get_tau_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field thrust from rotor_speed message
 *
 * @return [N] Total thrust
 */
static inline float mavlink_msg_rotor_speed_get_thrust(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a rotor_speed message into a struct
 *
 * @param msg The message to decode
 * @param rotor_speed C-struct to decode the message contents into
 */
static inline void mavlink_msg_rotor_speed_decode(const mavlink_message_t* msg, mavlink_rotor_speed_t* rotor_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rotor_speed->w_0 = mavlink_msg_rotor_speed_get_w_0(msg);
    rotor_speed->w_1 = mavlink_msg_rotor_speed_get_w_1(msg);
    rotor_speed->w_2 = mavlink_msg_rotor_speed_get_w_2(msg);
    rotor_speed->w_3 = mavlink_msg_rotor_speed_get_w_3(msg);
    rotor_speed->tau_x = mavlink_msg_rotor_speed_get_tau_x(msg);
    rotor_speed->tau_y = mavlink_msg_rotor_speed_get_tau_y(msg);
    rotor_speed->tau_z = mavlink_msg_rotor_speed_get_tau_z(msg);
    rotor_speed->thrust = mavlink_msg_rotor_speed_get_thrust(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROTOR_SPEED_LEN? msg->len : MAVLINK_MSG_ID_ROTOR_SPEED_LEN;
        memset(rotor_speed, 0, MAVLINK_MSG_ID_ROTOR_SPEED_LEN);
    memcpy(rotor_speed, _MAV_PAYLOAD(msg), len);
#endif
}
