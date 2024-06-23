/*
This code is used to control holybro X500 under Ubuntu 20.04 + ROS Noetic
The goal is to let the drone visit a few designated waypoints 
Vicon Mocap is used for indoor localization

06/16/2024 First version
06/22/2024 Code works (position control)

*/
#include <ros/ros.h>
#include "/home/cpsl/catkin_ws/src/dronestack/include/dronestack/offboard_ctrl.h"

// hardcode take-off position in case callback has not executed immediately (vicon)
// will store real take-off position from vicon after viconPose_cb is called
double vicon_posi_x = 0.0;
double vicon_posi_y = 0.0;
double vicon_posi_z = 0.4180;

bool vicon_data_stored = false;

int num_of_state_to_run = 5;

// mavros_msgs::SetMode land_mode;
// // land_mode.request.custom_mode = "AUTO.LAND";
// mavros_msgs::CommandTOL land_cmd;

int main(int argc, char **argv) {
    ros::init(argc, argv, "offboard_ctrl");
    ros::NodeHandle nh;

    // subscriber initialization
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &state_cb);
    current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &currentPose_cb);
    vicon_pose_sub = nh.subscribe<geometry_msgs::TransformStamped>("vicon/x500_1/x500_1", 10, &viconPose_cb);
    // current_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, & currentVelocity_cb, this);

    // publisher initialization
    pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    pos_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    // vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    thrust_pub = nh.advertise<mavros_msgs::Thrust>("mavros/setpoint_attitude/thrust", 10);

    // client service initialization (arming and mode)
    set_home_client = nh.serviceClient<mavros_msgs::CommandHome>("mavros/cmd/set_home");
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // publish position and velocity to vision pose for replacing GPS
    px4_vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1); 
    // px4_vision_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/vision_speed/speed_twist", 1);

    ros::Rate rate(50.0);   

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = vicon_posi_x;
    pose.pose.position.y = vicon_posi_y;
    pose.pose.position.z = vicon_posi_z + 0.5;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // initialize variables
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";    
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    // main loop
    while(ros::ok()) {
        
        // publish position to vision pose for replacing GPS (vicon) Must write this before switching mode and arming !
        px4_vision_pose_pub.publish(vision_pose_msg);

        // switch mode to offb_node (need to be inside the while loop to maintain offboard mode)
        handle_mode_switch(offb_set_mode, last_request);
        
        // arm the drone (need to be inside the while loop to maintain arming)
        handle_arm_command(arm_cmd, last_request);

        // major position controller 
        bool ifLand = position_control(last_request, arm_cmd);

        if(ifLand == true) {
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}


// Callback functions
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    // current_state stores the mode of the drone (eg, arm, disarm, etc)
    current_state = *msg;
    // ROS_INFO("Received Position x:%.2f, y:%.2f, z:%.2f ", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);

}

void  currentPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // local_position stores the current position and orientation of the drone
    current_pose = *msg;

    // ROS_INFO("Current Gazebo Position: x:%.2f, y:%.2f, z:%.2f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
}

void  viconPose_cb(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    // we only store translation and orientation quaternion once
    if (!vicon_data_stored) {
        vicon_posi_x = msg->transform.translation.x;
        vicon_posi_y = msg->transform.translation.y;
        vicon_posi_z = msg->transform.translation.z;

        ROS_INFO("Received Take-off Position from Vicon x:%.2f, y:%.2f, z:%.2f", vicon_posi_x, vicon_posi_y, vicon_posi_z);
    
        vicon_data_stored = true;
    }    
        
    vicon_pose = *msg;    
    
    // Create and publish pose message to vision
    // vision_pose_msg.header = msg->header;
    vision_pose_msg.header.frame_id = "map";
    vision_pose_msg.header.stamp = ros::Time::now();
    vision_pose_msg.pose.position.x = msg->transform.translation.x;
    vision_pose_msg.pose.position.y = msg->transform.translation.y;
    vision_pose_msg.pose.position.z = msg->transform.translation.z;
    vision_pose_msg.pose.orientation.x = msg->transform.rotation.x;
    vision_pose_msg.pose.orientation.y = msg->transform.rotation.y;
    vision_pose_msg.pose.orientation.z = msg->transform.rotation.z;
    vision_pose_msg.pose.orientation.w = msg->transform.rotation.w;

    // ROS_INFO("check vicon rotation: x:%.2f, y:%.2f, z:%.2f, w:%.2f", vision_pose_msg.pose.orientation.x, vision_pose_msg.pose.orientation.y, vision_pose_msg.pose.orientation.z, vision_pose_msg.pose.orientation.w);
    // px4_vision_pose_pub.publish(vision_pose_msg);    
}


bool position_control(ros::Time& last_request, mavros_msgs::CommandBool& arm_cmd) {

    // state 0 is landing state, state 1 is take off state, others are moving state
    if(state == 0) {
        target = Eigen::Vector3d(current_pose.pose.position.x, current_pose.pose.position.y, vicon_posi_z + 0.13);
    }
    else if(state == 1) { //take off
        target = Eigen::Vector3d(vicon_posi_x, vicon_posi_y, vicon_posi_z + 0.3);
    }
    else if(state == 2) {
        target = Eigen::Vector3d(vicon_posi_x, vicon_posi_y + 0.5, vicon_posi_z + 0.3);
    }
    else if(state == 3) {
        target = Eigen::Vector3d(vicon_posi_x + 0.5, vicon_posi_y + 0.5, vicon_posi_z + 0.3);
    }
    else if(state == 4) {
        target = Eigen::Vector3d(vicon_posi_x + 0.5, vicon_posi_y, vicon_posi_z + 0.3);
    }
    else if(state == 5) {
        target = Eigen::Vector3d(vicon_posi_x, vicon_posi_y, vicon_posi_z + 0.3);
    }

    target_pose.header.frame_id = "map";
    target_pose.header.stamp = ros::Time::now();
    target_pose.pose.position.x = target[0];
    target_pose.pose.position.y = target[1];
    target_pose.pose.position.z = target[2];
    target_pose.pose.orientation.x = 0;
    target_pose.pose.orientation.y = 0;
    target_pose.pose.orientation.z = 0;
    target_pose.pose.orientation.w = 0.91;

    // ROS_INFO("sent target Position x:%.2f, y:%.2f, z:%.2f", target_pose.pose.position.x , target_pose.pose.position.y , target_pose.pose.position.z);

    pos_pub.publish(target_pose); 
        
    // check if the drone reaches the target position. If it is true, drone moves to next position
    // use target for velocity controll in vicon. use target_trans otherwise



    if(isAtPosition(target[0], target[1], target[2], 0.1, 0.15)) {
        if(state == 0) {
            ROS_INFO("Reached Landing Position. Prepare to stop");

            return true;
        }
    }

    if(isAtPosition(target[0], target[1], target[2], 0.1, 0.15) && (ros::Time::now() - last_request > ros::Duration(5.0))) {
        
        // if the state is already the last task to run, set the state to zero so that the drone can land
        // it is safer to write ">=" than "=="
        if (state >= num_of_state_to_run) {
            state = 0;
            // cannot just terminate the program using return func here, as a safe landing (waypoint ctrl) is required beforehand
        }
        else  {
            state++;
        }

        last_request = ros::Time::now();
        ROS_INFO("Position x:%.2f, y:%.2f, z:%.2f reached. Moving to next desired position. State is:%.2d", target[0], target[1], target[2], state);            
        
    }

    return false;
}

void  handle_mode_switch(mavros_msgs::SetMode& mode_cmd, ros::Time& last_request) {
    // function to switch mode to "OFFBOARD"
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent) {
            ROS_INFO("OFFBOARD enabled");
            last_request = ros::Time::now();
        } else {
            ROS_WARN("Failed to set OFFBOARD mode");
        }
    } 
    // else {
    //     ROS_INFO("Current mode: %s", current_state.mode.c_str());
    // }
}

void  handle_arm_command(mavros_msgs::CommandBool& arm_cmd, ros::Time& last_request) {
    if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (arming_client.call(arm_cmd)) {
            if (arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
                last_request = ros::Time::now();
            } else {
                ROS_WARN("Failed to arm vehicle: response success = false");
            }
        } else {
            ROS_WARN("Failed to arm vehicle: service call failed");
        }
    } 
    // else {
    //     ROS_INFO("Current armed state: %s", current_state.armed ? "ARMED" : "DISARMED");
    // }
}


bool  isAtPosition(double x, double y, double z, double xy_offset, double z_offset) {
    // a function to check if the difference between current position and target position is within a predefined offset
    // ROS_INFO("current position | x:%.2f, y:%.2f, z:%.2f", local_position.pose.position.x, local_position.pose.position.y, local_position.pose.position.z);
    Eigen::Vector2d desired_posi_xy(x, y);

    // // (vicon)     
    // Eigen::Vector2d current_posi_xy(vicon_pose.transform.translation.x, vicon_pose.transform.translation.y);
    // return ((desired_posi_xy - current_posi_xy).norm() < xy_offset && abs(z - vicon_pose.transform.translation.z) < z_offset);

    // (gazebo)
    Eigen::Vector2d current_posi_xy(current_pose.pose.position.x, current_pose.pose.position.y);
    return ((desired_posi_xy - current_posi_xy).norm() < xy_offset && abs(z - current_pose.pose.position.z) < z_offset);
}

geometry_msgs::Quaternion rpy2quaternion(double roll, double pitch, double yaw)
{
  tf2::Quaternion quaternion_tf2;
  quaternion_tf2.setRPY(roll, pitch, yaw);
  geometry_msgs::Quaternion quaternion;
  quaternion.x = quaternion_tf2.x();
  quaternion.y = quaternion_tf2.y();
  quaternion.z = quaternion_tf2.z();
  quaternion.w = quaternion_tf2.w();
  return quaternion;
}













