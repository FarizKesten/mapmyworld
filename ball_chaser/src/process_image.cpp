#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include "sensor_msgs/Image.h"
#include "opencv2/highgui/highgui.hpp"

//Define a gloabl client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in
//he specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive
    // the robot
    ROS_INFO_STREAM("Send linear velocity: " << lin_x << " and angular velocity: " << ang_z);
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv)){
        ROS_ERROR("Failed to call service /command_robot");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    // Loop through each pixel in the image and check if there's
    // a bright white one. Then, identify if this pixel falls in the left,
    // mid, or right side of the image. Depending on the white ball position,
    // call the drive_bot function and pass velocities to it.
    // Request a stop when there's no white ball seen by the camera
    int white_pixel = 255;
    bool ball_found = false;
    int center_pixel = 0;
    int num_pixels = 0;

    for (int i = 0; i+2 < img.data.size(); i+=3)
    {
        // find white pixel
        if (img.data[i+0] == white_pixel &&
            img.data[i+1] == white_pixel &&
            img.data[i+2] == white_pixel)
            {
                ball_found = true;
                center_pixel += (i % img.step);
                num_pixels += 1;
            }
    }

    if (!ball_found)
    {
        ROS_WARN("No white ball found!");
        drive_robot(-0.1, 0); //reverse a bit to find the ball
        return;
    }

    center_pixel /= num_pixels * 3;
    if (center_pixel < 1 * img.width / 3)
        drive_robot(0.5,1); // turn counter-clockwise
    else if (center_pixel > 2 * img.width / 3)
        drive_robot(0.5,-1); //turn clockwise
    else
        drive_robot(1,0);
}



int main(int argc, char * argv[])
{
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the
    // process_image_callback
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();
    ros::shutdown();
    return 0;
}
