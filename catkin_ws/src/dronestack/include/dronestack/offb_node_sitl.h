#ifndef OFFB_NODE_SITL_H
#define OFFB_NODE_SITL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>

#include "dronestack/waypoint_nav.h" 

class OffboardControlSITL {
public:
    OffboardControlSITL();
    bool control(dronestack::waypoint_nav::Request &req, dronestack::waypoint_nav::Response &res);
    void control(const Eigen::Vector3d target_point);
    void control();
    Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw);
    ros::NodeHandle nh;
    ros::ServiceServer waypoint_server;

    ros::Subscriber state_sub, current_pose_sub, current_vel_sub, vicon_pose_sub;
    ros::Publisher pos_pub, vel_pub, px4_vision_pose_pub, px4_vision_vel_pub;
    ros::ServiceClient arming_client, set_mode_client, set_home_client;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped target_pose, current_pose, vision_pose_msg;
    geometry_msgs::TwistStamped vision_velocity_msg;
    geometry_msgs::Pose global_position;
    geometry_msgs::TwistStamped vel_msg;
    geometry_msgs::TransformStamped vicon_pose;

    

    Eigen::Vector3d target;
    Eigen::Vector3d target_trans;
    std::vector<int> tasks;
    int state;
    // int task_indicator;

    void initialization();
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void currentPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void currentVelocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void viconPose_cb(const geometry_msgs::TransformStamped::ConstPtr& msg);
    bool set_waypoint(dronestack::waypoint_nav::Request &req, dronestack::waypoint_nav::Response &res);
    void handle_mode_switch(mavros_msgs::SetMode& mode_cmd, ros::Time& last_request);
    void handle_arm_command(mavros_msgs::CommandBool& arm_cmd, ros::Time& last_request);
    bool isAtPosition(double x, double y, double z, double xy_offset, double z_offset);
    std::tuple<float, float, float> quaternionToRpy(float x, float y, float z, float w);
    geometry_msgs::TwistStamped limitVelocity(geometry_msgs::TwistStamped vel_msg, float max_velocity = 0.5);
    Eigen::Vector3d transformPoint(const Eigen::Vector3d &translation, const Eigen::Quaterniond &rotation, const Eigen::Vector3d &point);
private:

};

#endif // OFFB_NODE_SITL_H
