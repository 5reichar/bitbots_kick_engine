#include "ros_interface/publisher/controler_command_publisher.h"
#include <bitbots_msgs/JointCommand.h>

ControllerCommandPublisher::ControllerCommandPublisher(ros::NodeHandle & ros_node_handle)
{
    ros_publisher_controller_command_ = ros_node_handle.advertise<bitbots_msgs::JointCommand>("throw_motor_goals", 1);
}

void ControllerCommandPublisher::publish()
{
    //TODO: implement
}