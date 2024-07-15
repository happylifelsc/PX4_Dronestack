#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/ResilientStateEstimate.h>
#include <raw_rpm_ros/rpm_quad.h>
#include <offboard/combined_data.h>

geometry_msgs::PoseStamped ekf;
geometry_msgs::TransformStamped vicon;
sensor_msgs::Imu imu;
raw_rpm_ros::rpm_quad rpm4;
mavros_msgs::ResilientStateEstimate rse;


offboard::combined_data combined_data_msg;

ros::Subscriber ekf_sub, vicon_sub, imu_sub, rpm_sub, resilient_control_sub;
ros::Publisher combined_data_pub;


bool received_ekf = false;
bool received_vicon = false;
bool received_imu = false;
bool received_rpm = false;
bool received_rse = false;

void ekf_cb(const geometry_msgs::PoseStamped::ConstPtr& ekf_msg) {
    ekf = *ekf_msg;
    received_ekf = true;

    ROS_INFO("ekf: %.2f", ekf.pose.position.x);
}

void vicon_cb(const geometry_msgs::TransformStamped::ConstPtr& vicon_msg) {
    vicon = *vicon_msg;
    received_vicon = true;

    ROS_INFO("vicon: %.2f", vicon.transform.rotation.x);
}

void imu_cb(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    imu = *imu_msg;
    received_imu = true;
    
    ROS_INFO("imu: %.2f", imu.angular_velocity.x);
}

void rpm_cb(const raw_rpm_ros::rpm_quad::ConstPtr& rpm_msg) {
    rpm4 = *rpm_msg;
    received_rpm = true;

    ROS_INFO("rpm: %.2f", rpm4.rotor1);
}

void resilient_state_cb(const mavros_msgs::ResilientStateEstimate::ConstPtr& rse_msg) {
    rse = *rse_msg;
    received_rse = true;

    ROS_INFO("received rse");
}

void publish_combined_data(const geometry_msgs::PoseStamped ekf,
                      const geometry_msgs::TransformStamped vicon,
                      const sensor_msgs::Imu imu,
                      const raw_rpm_ros::rpm_quad rpm4, const mavros_msgs::ResilientStateEstimate rse) {

    combined_data_msg.ekf_orient_w = ekf.pose.orientation.w;
    combined_data_msg.ekf_orient_x = ekf.pose.orientation.x;
    combined_data_msg.ekf_orient_y = ekf.pose.orientation.y;
    combined_data_msg.ekf_orient_z = ekf.pose.orientation.z;
    combined_data_msg.vicon_posi_x = vicon.transform.translation.x;
    combined_data_msg.vicon_posi_y = vicon.transform.translation.y;
    combined_data_msg.vicon_posi_z = vicon.transform.translation.z;
    combined_data_msg.vicon_orient_w = vicon.transform.rotation.w;
    combined_data_msg.vicon_orient_x = vicon.transform.rotation.x;
    combined_data_msg.vicon_orient_y = vicon.transform.rotation.y;
    combined_data_msg.vicon_orient_z = vicon.transform.rotation.z;
    combined_data_msg.imu_ang_vel_x = imu.angular_velocity.x;
    combined_data_msg.imu_ang_vel_y = imu.angular_velocity.y;
    combined_data_msg.imu_ang_vel_z = imu.angular_velocity.z;
    combined_data_msg.imu_linear_accel_x = imu.linear_acceleration.x;
    combined_data_msg.imu_linear_accel_y = imu.linear_acceleration.y;
    combined_data_msg.imu_linear_accel_z = imu.linear_acceleration.z;
    combined_data_msg.rotor1 = rpm4.rotor1;
    combined_data_msg.rotor2 = rpm4.rotor2;
    combined_data_msg.rotor3 = rpm4.rotor3;
    combined_data_msg.rotor4 = rpm4.rotor4;

    combined_data_msg.rotor_speed = rse.rotor_speed;
    combined_data_msg.torque_ground_truth = rse.torque_ground_truth;
    combined_data_msg.thrust_ground_truth = rse.thrust_ground_truth;
    combined_data_msg.raw_angular_vel = rse.raw_angular_vel;
    combined_data_msg.raw_acc = rse.raw_acc;
    combined_data_msg.imu_angular_vel = rse.imu_angular_vel;
    combined_data_msg.imu_acc = rse.imu_acc;
    combined_data_msg.attitude_standard = rse.attitude_standard;
    combined_data_msg.torque_estimate = rse.torque_estimate;
    combined_data_msg.thrust_estimate = rse.thrust_estimate;
    combined_data_msg.attitude_estimate = rse.attitude_estimate;
    combined_data_msg.angular_vel_estimate = rse.angular_vel_estimate;
    combined_data_msg.anomaly_score = rse.anomaly_score;
    combined_data_msg.anomaly_detection_rate = rse.anomaly_detection_rate;
    combined_data_msg.anomaly_flag = rse.anomaly_flag;
    combined_data_msg.imu_acc_offset = rse.imu_acc_offset;
    combined_data_msg.imu_gyro_offset = rse.imu_gyro_offset;
    combined_data_msg.attack_flag = rse.attack_flag;



    ROS_INFO("Check ekf.pose.orientation.x in publish function %.2f", ekf.pose.orientation.x);
    combined_data_pub.publish(combined_data_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "combined_data");
    ros::NodeHandle nh;

    ekf_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &ekf_cb);
    vicon_sub = nh.subscribe<geometry_msgs::TransformStamped>("vicon/x500_1/x500_1", 10, &vicon_cb);
    imu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &imu_cb);
    rpm_sub = nh.subscribe<raw_rpm_ros::rpm_quad>("rpm_quad", 10, &rpm_cb);
    resilient_control_sub = nh.subscribe<mavros_msgs::ResilientStateEstimate>("/mavros/resilient_state_estimate", 10, &resilient_state_cb);


    combined_data_pub = nh.advertise<offboard::combined_data>("combined_data", 10);
    
    ros::Rate rate(50.0);   

    while(ros::ok()) {

        // if(received_ekf == true && received_vicon == true && received_imu == true && received_rpm == true) {
        publish_combined_data(ekf, vicon, imu, rpm4, rse);

        received_ekf = false;
        received_vicon = false;
        received_imu = false;
        received_rpm = false;

        // }
        ros::spinOnce();
        rate.sleep();
    
    }

    return 0;
}





