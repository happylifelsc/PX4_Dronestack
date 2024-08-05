#ifndef RESILIENT_STATE_ESTIMATE_HPP
#define RESILIENT_STATE_ESTIMATE_HPP

#include <uORB/topics/resilient_state_estimate.h>

class MavlinkStreamResilientStateEstimate : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamResilientStateEstimate(mavlink); }

    static constexpr const char *get_name_static() { return "RESILIENT_STATE_ESTIMATE"; }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE; }

    const char *get_name() const override { return get_name_static(); }
    uint16_t get_id() override { return get_id_static(); }

    unsigned get_size() override
    {
        return MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    explicit MavlinkStreamResilientStateEstimate(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    uORB::Subscription _resilient_state_estimate_sub{ORB_ID(resilient_state_estimate)};

    bool send() override
    {
        resilient_state_estimate_s resilient_state_estimate_msg;

        if (_resilient_state_estimate_sub.update(&resilient_state_estimate_msg)) {
            mavlink_resilient_state_estimate_t msg{};

            msg.time_boot_ms = resilient_state_estimate_msg.timestamp / 1000;

            // Copying rotor_speed array elements
            for (int i = 0; i < 4; i++) {
                msg.rotor_speed[i] = resilient_state_estimate_msg.rotor_speed[i];
            }

            // Copying torque_ground_truth array elements
            for (int i = 0; i < 3; i++) {
                msg.torque_ground_truth[i] = resilient_state_estimate_msg.torque_ground_truth[i];
            }

            msg.thrust_ground_truth = resilient_state_estimate_msg.thrust_ground_truth;

            // Copying imu_angular_vel array elements
            for (int i = 0; i < 3; i++) {
                msg.raw_angular_vel[i] = resilient_state_estimate_msg.raw_angular_vel[i];
                msg.raw_acc[i] = resilient_state_estimate_msg.raw_acc[i];
                msg.imu_angular_vel[i] = resilient_state_estimate_msg.imu_angular_vel[i];
                msg.imu_acc[i] = resilient_state_estimate_msg.imu_acc[i];
                msg.angular_vel_estimate[i] = resilient_state_estimate_msg.angular_vel_estimate[i];
            }

            // Copying attitude_standard array elements (quaternion)
            for (int i = 0; i < 4; i++) {
                msg.attitude_standard[i] = resilient_state_estimate_msg.attitude_standard[i];
                msg.attitude_estimate[i] = resilient_state_estimate_msg.attitude_estimate[i];
            }

            // Copying torque_estimate array elements
            for (int i = 0; i < 3; i++) {
                msg.torque_estimate[i] = resilient_state_estimate_msg.torque_estimate[i];
            }

            msg.thrust_estimate = resilient_state_estimate_msg.thrust_estimate;
            msg.anomaly_score = resilient_state_estimate_msg.anomaly_score;
            msg.anomaly_detection_rate = resilient_state_estimate_msg.anomaly_detection_rate;
            msg.anomaly_flag = resilient_state_estimate_msg.anomaly_flag;

            // Attacks
            for (int i = 0; i < 3; i++) {
                msg.imu_acc_offset[i] = resilient_state_estimate_msg.imu_acc_offset[i];
                msg.imu_gyro_offset[i] = resilient_state_estimate_msg.imu_gyro_offset[i];
            }
            msg.attack_flag = resilient_state_estimate_msg.attack_flag;

            mavlink_msg_resilient_state_estimate_send_struct(_mavlink->get_channel(), &msg);

            return true;
        }

        return false;
    }
};

#endif // RESILIENT_STATE_ESTIMATE_HPP
