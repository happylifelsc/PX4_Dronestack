#ifndef ROTOR_SPEED_HPP
#define ROTOR_SPEED_HPP

#include <uORB/topics/rpm.h>

class MavlinkStreamRotorSpeed : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamRotorSpeed(mavlink); }

    static constexpr const char *get_name_static() { return "ROTOR_SPEED"; }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ROTOR_SPEED; }

    const char *get_name() const override { return get_name_static(); }
    uint16_t get_id() override { return get_id_static(); }

    unsigned get_size() override
    {
        return _rpm_subs.advertised_count() * (MAVLINK_MSG_ID_ROTOR_SPEED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
    }

private:
    explicit MavlinkStreamRotorSpeed(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    uORB::SubscriptionMultiArray<rpm_s> _rpm_subs{ORB_ID::rpm};

    bool send() override
    {
        bool updated = false;
        rpm_s rpm_data[4] = {};

        for (int i = 0; i < _rpm_subs.size(); i++) {
            if (_rpm_subs[i].update(&rpm_data[i])) {
                updated = true;
            }
        }

        if (updated) {
            mavlink_rotor_speed_t msg{};

            msg.w_0 = 0.3f;
            msg.w_1 = 0.3f;
            msg.w_2 = 0.3f;
            msg.w_3 = 0.3f;

            // msg.w_0 = rpm_data[0].indicated_frequency_rpm;
            // msg.w_1 = rpm_data[1].indicated_frequency_rpm;
            // msg.w_2 = rpm_data[2].indicated_frequency_rpm;
            // msg.w_3 = rpm_data[3].indicated_frequency_rpm;

            // Assuming tau_x, tau_y, tau_z, and thrust are calculated or available
            msg.tau_x = 0.0f; // Replace with actual value or calculation
            msg.tau_y = 0.0f; // Replace with actual value or calculation
            msg.tau_z = 0.0f; // Replace with actual value or calculation
            msg.thrust = 0.0f; // Replace with actual value or calculation

            mavlink_msg_rotor_speed_send_struct(_mavlink->get_channel(), &msg);

            return true;
        }

        return false;
    }
};

#endif // ROTOR_SPEED_HPP



// #ifndef ROTOR_SPEED_HPP
// #define ROTOR_SPEED_HPP

// #include <uORB/topics/rpm.h>

// class MavlinkStreamRotorSpeed : public MavlinkStream
// {
// public:
// 	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamRotorSpeed(mavlink); }

// 	static constexpr const char *get_name_static() { return "ROTOR_SPEED"; }
// 	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ROTOR_SPEED; }

// 	const char *get_name() const override { return get_name_static(); }
// 	uint16_t get_id() override { return get_id_static(); }

// 	unsigned get_size() override
// 	{
// 		return (_rpm_sub[0].advertised() || _rpm_sub[1].advertised() || _rpm_sub[2].advertised() || _rpm_sub[3].advertised())
// 			? MAVLINK_MSG_ID_ROTOR_SPEED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
// 	}

// private:
// 	explicit MavlinkStreamRotorSpeed(Mavlink *mavlink) : MavlinkStream(mavlink) {}

// 	uORB::Subscription _rpm_sub[4]{
// 		{ORB_ID(rpm), 0},
// 		{ORB_ID(rpm), 1},
// 		{ORB_ID(rpm), 2},
// 		{ORB_ID(rpm), 3}
// 	};

// 	bool send() override
// 	{
// 		rpm_s rpm_data[4];
// 		bool updated = false;

// 		for (int i = 0; i < 4; ++i) {
// 			if (_rpm_sub[i].update(&rpm_data[i])) {
// 				updated = true;
// 			}
// 		}

// 		if (updated) {
// 			mavlink_rotor_speed_t msg{};

// 			msg.w_0 = rpm_data[0].indicated_frequency_rpm;
// 			msg.w_1 = rpm_data[1].indicated_frequency_rpm;
// 			msg.w_2 = rpm_data[2].indicated_frequency_rpm;
// 			msg.w_3 = rpm_data[3].indicated_frequency_rpm;

// 			// Assuming tau_x, tau_y, tau_z, and thrust are calculated or available
// 			msg.tau_x = 0.0f; // Replace with actual value or calculation
// 			msg.tau_y = 0.0f; // Replace with actual value or calculation
// 			msg.tau_z = 0.0f; // Replace with actual value or calculation
// 			msg.thrust = 0.0f; // Replace with actual value or calculation

// 			mavlink_msg_rotor_speed_send_struct(_mavlink->get_channel(), &msg);

// 			return true;
// 		}

// 		return false;
// 	}
// };

// #endif // ROTOR_SPEED_HPP
