#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
#include <std_msgs/Float64.h>

// ROS::Publisher
ros::Publisher motor_command_publisher;

// Create a handle_drive_request callback function that executes whenever a
// drive_bot service is requested. This function should publish the requested
// linear x and angular velocities to the robot wheel joints. After publishing
// the requested velocities, a message feedback should be returned with the
// requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
                          ball_chaser::DriveToTarget::Response& res)
{
    ROS_INFO("DriveToTargetRequest received - linear.x: %1.2f, angular.z: %1.2f",
             (float)req.linear_x, (float)req.angular_z);

    geometry_msgs::Twist motor_command;

    motor_command.angular.x = 0.0;
    motor_command.angular.y = 0.0;
    motor_command.angular.z = (double)req.angular_z;

    motor_command.linear.x  = (double)req.linear_x;
    motor_command.linear.y  = 0.0;
    motor_command.linear.z  = 0.0;

    motor_command_publisher.publish(motor_command);

    //return a response message
    res.msg_feedback = " linear velocity x: " + std::to_string(req.linear_x) +
                       " angular velocity z: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}



int main(int argc, char * argv[])
{
    ros::init(argc, argv, "drive_bot");

    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type
    // geometry_msgs::Twist on the robot actuation topic with a publishing
    // queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define a drive /ball_chaser/command_robot service with a handle_drive_request
    // callback function
    ros::ServiceServer service;
    service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    ROS_INFO("Ready to send joint commands");
    ros::spin();

    ros::shutdown();
    return 0;
}








