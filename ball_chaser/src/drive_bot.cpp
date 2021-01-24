#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"//Include the ball_chaser "DriveToTarget" header file OK
#include<stdlib.h>
#include<vector>

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// Handle_Drive class is callback function that executes whenever a drive_bot service is requested.
// This function should publish the requested linear x and angular velocities to the robot wheel joints, after publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

class Handle_Drive
{

   public:

   bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
   {
      ROS_INFO("DriveToTargetRequest received - linear_x:%1.2f, angular_z:%1.2f", (float)req.linear_x, (float)req.angular_z);
       // publish the requested linear x and angular velocities to the robot wheel joints
       // Create a motor_command object of type geometry_msgs::Twist
       geometry_msgs::Twist motor_command;
       // Set wheel velocities, forward request velocities.
       motor_command.linear.x = req.linear_x;
       motor_command.angular.z = req.angular_z;
       // Publish angles to drive the robot
       motor_command_publisher.publish(motor_command);

       // After publishing the requested velocities, a message feedback is returned with the  requested wheel velocities
       res.msg_feedback = "Wheel velocities set - linear_x: " + std::to_string(req.linear_x) + " , angular_z: " + std::to_string(req.angular_z);
       ROS_INFO_STREAM(res.msg_feedback);
       return true;
   }
};


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");
    
    Handle_Drive handle_drive;

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    //Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function

    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", 
&Handle_Drive::handle_drive_request, &handle_drive);
    ROS_INFO("Ready to send motor commands");

    //Handle ROS communication events
    ros::spin();

    return 0;
}
