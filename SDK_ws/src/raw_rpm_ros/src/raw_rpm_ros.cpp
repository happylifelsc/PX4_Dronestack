// ~/catkin_ws/src/my_mavros_bridge/src/raw_rpm_bridge.cpp
#include <ros/ros.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/RawRpm.h> // Assuming this message type exists
#include <raw_rpm_ros/rpm_quad.h>


ros::Subscriber raw_rpm_sub;
ros::Publisher rpm_quad_pub;

raw_rpm_ros::rpm_quad rpm_quad_msg;

void rpmAssembleCallback(const mavros_msgs::RawRpm::ConstPtr& msg) {


    if(msg->index == 0) rpm_quad_msg.rotor1 = msg->frequency;
    else if(msg->index == 1) rpm_quad_msg.rotor2 = msg->frequency;
    else if(msg->index == 2) rpm_quad_msg.rotor3 = msg->frequency;
    else if(msg->index == 3) rpm_quad_msg.rotor4 = msg->frequency;
    

    
    if(rpm_quad_msg.rotor1 != -1.0 && rpm_quad_msg.rotor2 != -1.0 && rpm_quad_msg.rotor3 != -1.0 && rpm_quad_msg.rotor4 != -1.0) {
        rpm_quad_pub.publish(rpm_quad_msg);

        rpm_quad_msg.rotor1 = -1.0;
        rpm_quad_msg.rotor2 = -1.0;
        rpm_quad_msg.rotor3 = -1.0;
        rpm_quad_msg.rotor4 = -1.0;
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rpm_quad");
    ros::NodeHandle nh;

    rpm_quad_pub = nh.advertise<raw_rpm_ros::rpm_quad>("rpm_quad", 10);
    raw_rpm_sub = nh.subscribe("/mavros/raw_rpm", 10, rpmAssembleCallback);

    // rotor_speed_msg->tau_x = rpm.tau_x;
    // rotor_speed_msg->tau_y = rpm.tau_y;
    // rotor_speed_msg->tau_z = rpm.tau_z;
    // rotor_speed_msg->thrust = rpm.thrust;

    ros::spin();
    return 0;
}

