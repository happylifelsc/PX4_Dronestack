#ifndef RESILIENT_STATE_ESTIMATE_HPP
#define RESILIENT_STATE_ESTIMATE_HPP


// Streaming out RESILIENT_STATE_ESTIMATE for sending resilient state estimates to ROS
class MavlinkStreamResilientStateEstimate : public MavlinkStream
{
public:
    const char *get_name() const override
    {
        return MavlinkStreamResilientStateEstimate::get_name_static();
    }

    static constexpr const char *get_name_static()
    {
        return "RESILIENT_STATE_ESTIMATE";
    }

    static constexpr uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE;
    }

    uint16_t get_id() override
    {
        return get_id_static();
    }

    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamResilientStateEstimate(mavlink);
    }

    unsigned get_size() override
    {
        // Assuming each subscription has similar logic to check if they are advertised
        return MAVLINK_MSG_ID_RESILIENT_STATE_ESTIMATE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    uORB::Subscription _resilient_state_estimate_sub{ORB_ID(resilient_state_estimate)};


    /* do not allow to copy this class */
    MavlinkStreamResilientStateEstimate(MavlinkStreamResilientStateEstimate &) = delete;
    MavlinkStreamResilientStateEstimate &operator = (const MavlinkStreamResilientStateEstimate &) = delete;

protected:
    explicit MavlinkStreamResilientStateEstimate(Mavlink *mavlink) : MavlinkStream(mavlink)
    {}

    bool send() override
    {
	// Gazebo rotor speeds, torques and thrusts
	resilient_state_estimate_s resilient_state_estimate_msg;

        if (_resilient_state_estimate_sub.update(&resilient_state_estimate_msg)) {
	    mavlink_resilient_state_estimate_t msg{};

		// Assuming the structure of the MAVLink message and the uORB message are defined as follows
		// and that both have been properly initialized or populated before this code.

		msg.time_boot_ms = resilient_state_estimate_msg.timestamp / 1000;

		// Copying rotor_speed array elements
		msg.rotor_speed[0] = resilient_state_estimate_msg.rotor_speed[0];
		msg.rotor_speed[1] = resilient_state_estimate_msg.rotor_speed[1];
		msg.rotor_speed[2] = resilient_state_estimate_msg.rotor_speed[2];
		msg.rotor_speed[3] = resilient_state_estimate_msg.rotor_speed[3];

		// Copying torque_ground_truth array elements
		msg.torque_ground_truth[0] = resilient_state_estimate_msg.torque_ground_truth[0];
		msg.torque_ground_truth[1] = resilient_state_estimate_msg.torque_ground_truth[1];
		msg.torque_ground_truth[2] = resilient_state_estimate_msg.torque_ground_truth[2];

		msg.thrust_ground_truth = resilient_state_estimate_msg.thrust_ground_truth;

		// Copying imu_angular_vel array elements
		msg.raw_angular_vel[0] = resilient_state_estimate_msg.raw_angular_vel[0];
		msg.raw_angular_vel[1] = resilient_state_estimate_msg.raw_angular_vel[1];
		msg.raw_angular_vel[2] = resilient_state_estimate_msg.raw_angular_vel[2];

		// Copying imu_angular_vel array elements
		msg.raw_acc[0] = resilient_state_estimate_msg.raw_acc[0];
		msg.raw_acc[1] = resilient_state_estimate_msg.raw_acc[1];
		msg.raw_acc[2] = resilient_state_estimate_msg.raw_acc[2];

		// Copying imu_angular_vel array elements
		msg.imu_angular_vel[0] = resilient_state_estimate_msg.imu_angular_vel[0];
		msg.imu_angular_vel[1] = resilient_state_estimate_msg.imu_angular_vel[1];
		msg.imu_angular_vel[2] = resilient_state_estimate_msg.imu_angular_vel[2];

		// Copying imu_angular_vel array elements
		msg.imu_acc[0] = resilient_state_estimate_msg.imu_acc[0];
		msg.imu_acc[1] = resilient_state_estimate_msg.imu_acc[1];
		msg.imu_acc[2] = resilient_state_estimate_msg.imu_acc[2];

		// Copying attitude_standard array elements (quaternion)
		msg.attitude_standard[0] = resilient_state_estimate_msg.attitude_standard[0];
		msg.attitude_standard[1] = resilient_state_estimate_msg.attitude_standard[1];
		msg.attitude_standard[2] = resilient_state_estimate_msg.attitude_standard[2];
		msg.attitude_standard[3] = resilient_state_estimate_msg.attitude_standard[3];

		// Copying torque_estimate array elements
		msg.torque_estimate[0] = resilient_state_estimate_msg.torque_estimate[0];
		msg.torque_estimate[1] = resilient_state_estimate_msg.torque_estimate[1];
		msg.torque_estimate[2] = resilient_state_estimate_msg.torque_estimate[2];

		msg.thrust_estimate = resilient_state_estimate_msg.thrust_estimate;

		// Copying attitude_estimate array elements (quaternion)
		msg.attitude_estimate[0] = resilient_state_estimate_msg.attitude_estimate[0];
		msg.attitude_estimate[1] = resilient_state_estimate_msg.attitude_estimate[1];
		msg.attitude_estimate[2] = resilient_state_estimate_msg.attitude_estimate[2];
		msg.attitude_estimate[3] = resilient_state_estimate_msg.attitude_estimate[3];

		// Copying angular_vel_estimate array elements
		msg.angular_vel_estimate[0] = resilient_state_estimate_msg.angular_vel_estimate[0];
		msg.angular_vel_estimate[1] = resilient_state_estimate_msg.angular_vel_estimate[1];
		msg.angular_vel_estimate[2] = resilient_state_estimate_msg.angular_vel_estimate[2];

		msg.anomaly_score = resilient_state_estimate_msg.anomaly_score;
		msg.anomaly_detection_rate = resilient_state_estimate_msg.anomaly_detection_rate;
		msg.anomaly_flag = resilient_state_estimate_msg.anomaly_flag;

		// Attacks
		msg.imu_acc_offset[0] = resilient_state_estimate_msg.imu_acc_offset[0];
		msg.imu_acc_offset[1] = resilient_state_estimate_msg.imu_acc_offset[1];
		msg.imu_acc_offset[2] = resilient_state_estimate_msg.imu_acc_offset[2];

		msg.imu_gyro_offset[0] = resilient_state_estimate_msg.imu_gyro_offset[0];
		msg.imu_gyro_offset[1] = resilient_state_estimate_msg.imu_gyro_offset[1];
		msg.imu_gyro_offset[2] = resilient_state_estimate_msg.imu_gyro_offset[2];
		msg.attack_flag = resilient_state_estimate_msg.attack_flag;


            mavlink_msg_resilient_state_estimate_send_struct(_mavlink->get_channel(), &msg);

            return true;
        }

        return false;
    }
};

#endif //RESILIENT_STATE_ESTIMATE_HPP
