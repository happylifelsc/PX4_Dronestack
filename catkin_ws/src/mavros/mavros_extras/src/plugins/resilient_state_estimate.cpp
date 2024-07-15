#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>

#include <mavros_msgs/ResilientStateEstimate.h> // Include your custom message header

namespace mavros {
namespace extra_plugins {

class ResilientStateEstimatePlugin : public plugin::PluginBase {
public:
    ResilientStateEstimatePlugin() : PluginBase(),
        nh("~")
    { };

    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);

        // Set up the ROS publisher
        resilient_state_estimate_pub = nh.advertise<mavros_msgs::ResilientStateEstimate>("resilient_state_estimate", 10);
    };

    // Subscription for incoming MAVLink messages from PX4
    Subscriptions get_subscriptions()
    {
        return {
            make_handler(&ResilientStateEstimatePlugin::handle_resilient_state_estimate_message)
        };
    }

private:
    ros::NodeHandle nh;
    ros::Publisher resilient_state_estimate_pub;

    // Handler for the custom MAVLink message
    void handle_resilient_state_estimate_message(const mavlink::mavlink_message_t *msg, mavlink::common::msg::RESILIENT_STATE_ESTIMATE &resilient_state_estimate)
    {
        auto estimate_msg = boost::make_shared<mavros_msgs::ResilientStateEstimate>();
        estimate_msg->header.stamp = m_uas->synchronise_stamp(static_cast<uint64_t>(resilient_state_estimate.time_boot_ms * 1000ULL));

        estimate_msg->attitude_standard.w = resilient_state_estimate.attitude_standard[0];
        estimate_msg->attitude_standard.x = resilient_state_estimate.attitude_standard[1]; 
        estimate_msg->attitude_standard.y = resilient_state_estimate.attitude_standard[2];
        estimate_msg->attitude_standard.z = resilient_state_estimate.attitude_standard[3];
        
        estimate_msg->attitude_estimate.w = resilient_state_estimate.attitude_estimate[0];
        estimate_msg->attitude_estimate.x = resilient_state_estimate.attitude_estimate[1];
        estimate_msg->attitude_estimate.y = resilient_state_estimate.attitude_estimate[2];
        estimate_msg->attitude_estimate.z = resilient_state_estimate.attitude_estimate[3];

        for (int i = 0; i < 4; ++i) {
            estimate_msg->rotor_speed[i] = resilient_state_estimate.rotor_speed[i];
        }       


        for (int i = 0; i < 3; ++i) {
            estimate_msg->torque_ground_truth[i] = resilient_state_estimate.torque_ground_truth[i];
            estimate_msg->raw_angular_vel[i] = resilient_state_estimate.raw_angular_vel[i];
            estimate_msg->raw_acc[i] = resilient_state_estimate.raw_acc[i];
            estimate_msg->imu_angular_vel[i] = resilient_state_estimate.imu_angular_vel[i];
            estimate_msg->imu_acc[i] = resilient_state_estimate.imu_acc[i];
            estimate_msg->torque_estimate[i] = resilient_state_estimate.torque_estimate[i];
            estimate_msg->angular_vel_estimate[i] = resilient_state_estimate.angular_vel_estimate[i];
            estimate_msg->imu_acc_offset[i] = resilient_state_estimate.imu_acc_offset[i];
            estimate_msg->imu_gyro_offset[i] = resilient_state_estimate.imu_gyro_offset[i];
        }
        estimate_msg->thrust_ground_truth = resilient_state_estimate.thrust_ground_truth;
        estimate_msg->thrust_estimate = resilient_state_estimate.thrust_estimate;
        estimate_msg->anomaly_score = resilient_state_estimate.anomaly_score;
        estimate_msg->anomaly_detection_rate = resilient_state_estimate.anomaly_detection_rate;
        estimate_msg->anomaly_flag = resilient_state_estimate.anomaly_flag;
        estimate_msg->attack_flag = resilient_state_estimate.attack_flag;
        // Publish the custom ROS message
        resilient_state_estimate_pub.publish(estimate_msg);
    }

};
}   // namespace extra_plugins
}   // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ResilientStateEstimatePlugin, mavros::plugin::PluginBase)






