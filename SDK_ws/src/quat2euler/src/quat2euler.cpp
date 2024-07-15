/* quaternion to rpy FOR Drone and Vicon*/

#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include <iostream>

#define PI 3.1415926

ros::Publisher droneRpyPub;
ros::Publisher viconRpyPub;

geometry_msgs::Transform vicon_pose;
geometry_msgs::Quaternion drone_pose;

void droneCvtCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    //    std::cout<<"Im here"<<std::endl;
    drone_pose = msg->pose.orientation;
    geometry_msgs::Twist droneRPY;
    double roll, pitch, yaw;

    tf::Quaternion q(
        drone_pose.x,
        drone_pose.y,
        drone_pose.z,
        drone_pose.w);
    tf::Matrix3x3 m(q);
    
    m.getRPY(roll, pitch, yaw);  
    droneRPY.angular.x = roll;
    droneRPY.angular.y = pitch;
    droneRPY.angular.z = yaw;   
    std::cout<<"Drone Yaw= "<<yaw<<" rad OR "<<yaw*180.0/PI<<" degree        "<<"Pitch= "<<pitch*180.0/PI<<" degree, "<<"Roll= "<<roll*180.0/PI<<" degree"<<std::endl; 
    
    viconRpyPub.publish(droneRPY);
}

void viconCvtCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
    
    vicon_pose = msg->transform;
    geometry_msgs::Twist viconRPY;
    double roll, pitch, yaw;
    
    tf::Quaternion q(
        vicon_pose.rotation.x,
        vicon_pose.rotation.y,
        vicon_pose.rotation.z,
        vicon_pose.rotation.w);
    tf::Matrix3x3 m(q);
    
    m.getRPY(roll, pitch, yaw);  
    viconRPY.angular.x = roll;
    viconRPY.angular.y = pitch;
    viconRPY.angular.z = yaw;

    std::cout<<"Vicon Yaw= "<<yaw<<" rad OR "<<yaw*180.0/PI<<" degree        "<<"Pitch= "<<pitch*180.0/PI<<" degree, "<<"Roll= "<<roll*180.0/PI<<" degree"<<std::endl; 
    
    viconRpyPub.publish(viconRPY);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quat2euler_node");
    
    ros::NodeHandle nh;    
    
    ros::Subscriber viconSub = nh.subscribe("vicon/x500_1/x500_1", 10, viconCvtCallback);
    ros::Subscriber droneImuSub = nh.subscribe("/mavros/local_position/pose", 10, droneCvtCallback);

    droneRpyPub = nh.advertise<geometry_msgs::Twist>("debug_droneRPY", 10);
    viconRpyPub = nh.advertise<geometry_msgs::Twist>("debug_viconRPY", 10);
    
    ros::spin();
    
    return 0;
}
