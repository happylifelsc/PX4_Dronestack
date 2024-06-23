#ifndef OFFB_NODE_SITL_H
#define OFFB_NODE_SITL_H

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <eigen3/Eigen/Dense>


// void control(const Eigen::Vector3d target_point);
bool position_control(ros::Time& last_request, mavros_msgs::CommandBool& arm_cmd);
Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw);


ros::Subscriber state_sub, current_pose_sub, current_vel_sub, vicon_pose_sub;
ros::Publisher pos_pub, pos_raw_pub, vel_pub, thrust_pub, px4_vision_pose_pub, px4_vision_vel_pub;
ros::ServiceClient arming_client, set_mode_client, set_home_client;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped target_pose, current_pose, vision_pose_msg;
geometry_msgs::TwistStamped vision_velocity_msg;
geometry_msgs::TwistStamped vel_msg;
geometry_msgs::TransformStamped vicon_pose;



Eigen::Vector3d target;
Eigen::Vector3d target_trans;
mavros_msgs::Thrust thrust_msg;

std::vector<int> tasks;
int state = 1; // should not modify

void state_cb(const mavros_msgs::State::ConstPtr& msg);
void currentPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void viconPose_cb(const geometry_msgs::TransformStamped::ConstPtr& msg);
void handle_mode_switch(mavros_msgs::SetMode& mode_cmd, ros::Time& last_request);
void handle_arm_command(mavros_msgs::CommandBool& arm_cmd, ros::Time& last_request);
bool isAtPosition(double x, double y, double z, double xy_offset, double z_offset);
geometry_msgs::Quaternion rpy2quaternion(double roll, double pitch, double yaw);

#endif // OFFB_NODE_SITL_H
