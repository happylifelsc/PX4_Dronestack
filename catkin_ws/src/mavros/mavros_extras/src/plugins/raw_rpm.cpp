#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>

#include <mavros_msgs/RawRpm.h> // Include your custom message header

namespace mavros {
namespace extra_plugins {

class RawRpmPlugin : public plugin::PluginBase {
public:
    RawRpmPlugin() : PluginBase(),
        nh("~")
    { };

    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);
        
        // Set up the ROS publisher
        rpm_pub = nh.advertise<mavros_msgs::RawRpm>("raw_rpm", 10);
    };

    // Subscription for incoming MAVLink messages from PX4
    Subscriptions get_subscriptions()
    {
        return {
            make_handler(&RawRpmPlugin::handle_raw_rpm)
        };
    }

private:
    ros::NodeHandle nh;
    ros::Publisher rpm_pub;

    // Handler for the custom MAVLink message
    void handle_raw_rpm(const mavlink::mavlink_message_t *msg, mavlink::common::msg::RAW_RPM &rpm)
    {
        auto rpm_msg = boost::make_shared<mavros_msgs::RawRpm>();
        // rotor_speed_msg->header.stamp = m_uas->synchronise_stamp(rotor_speed.header.stamp);

        // Populate the RotorSpeed message fields with the received MAVLink message data
        rpm_msg->index = rpm.index;
        rpm_msg->frequency = rpm.frequency;

        // rotor_speed_msg->tau_x = rpm.tau_x;
        // rotor_speed_msg->tau_y = rpm.tau_y;
        // rotor_speed_msg->tau_z = rpm.tau_z;
        // rotor_speed_msg->thrust = rpm.thrust;

        // Publish the custom ROS message
        rpm_pub.publish(rpm_msg);
    }

};
}   // namespace extra_plugins
}   // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::RawRpmPlugin, mavros::plugin::PluginBase)
