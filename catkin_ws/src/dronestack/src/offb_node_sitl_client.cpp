#include "ros/ros.h"
#include "dronestack/waypoint_nav.h"
#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "waypoint_nav_client");

    if (argc != 1) {
        ROS_INFO("usage: waypoint_nav_client");
        return 1;
    }

    ros::NodeHandle nh;
    ros::ServiceClient waypoint_client = nh.serviceClient<dronestack::waypoint_nav>("waypoint_nav");

    while (ros::ok()) {
        double x, y, z;
        std::cout << "Enter next waypoint (X Y Z): ";
        std::cin >> x >> y >> z;

        dronestack::waypoint_nav srv;
        srv.request.a = x;
        srv.request.b = y;
        srv.request.c = z;

        if (waypoint_client.call(srv)) {
            ROS_INFO("Waypoint sent: X=%f, Y=%f, Z=%f", srv.request.a, srv.request.b, srv.request.c);
            ROS_INFO("Success: %d", srv.response.success);
        } else {
            ROS_ERROR("Failed to call service waypoint_nav");
            return 1;
        }

        std::string user_input;
        std::cout << "Send another waypoint? (y/n): ";
        std::cin >> user_input;
        if (user_input != "y") {
            break;
        }
    }

    return 0;
}




// #include "ros/ros.h"
// #include "dronestack/offb_node_sitl.h"
// #include "dronestack/waypoint_nav.h"
// #include <cstdlib>

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "waypoint_nav_client");
//     if (argc != 4)
//     {
//     ROS_INFO("usage: waypoint_nav_client X Y Z");
//     return 1;
//     }

//     ros::NodeHandle nh;
//     ros::ServiceClient waypoint_client = nh.serviceClient<dronestack::waypoint_nav>("waypoint_nav");
//     dronestack::waypoint_nav srv; // the type might need to be changed according to how you created the ROS service (not done yet).

//     // store inputs fro terminal
//     srv.request.a = atof(argv[1]);
//     srv.request.b = atof(argv[2]);
//     srv.request.c = atof(argv[3]);


//     if (waypoint_client.call(srv)) {
//         ROS_INFO("Success: %d", srv.response.success);
//     }
//     else {
//         ROS_ERROR("Failed to call service add_two_ints");
//         return 1;
//     }

//     return 0;

// }