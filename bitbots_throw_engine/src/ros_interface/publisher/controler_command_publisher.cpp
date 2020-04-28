#include "ros_interface/publisher/controler_command_publisher.h"
#include <bitbots_msgs/JointCommand.h>

ControllerCommandPublisher::ControllerCommandPublisher(ros::NodeHandle & ros_node_handle, std::string topic)
{
    ros_publisher_controller_command_ = ros_node_handle.advertise<bitbots_msgs::JointCommand>(topic, 1);
}

void ControllerCommandPublisher::publish(ros::Time time, std::vector<std::string> joint_names, std::vector<double> positions)
{
    //TODO: implement

    publish(time, joint_names, positions, std::vector<double>());
}

void ControllerCommandPublisher::publish(ros::Time time, std::vector<std::string> joint_names, std::vector<double> positions, std::vector<double> velocities)
{
    //TODO: implement

    publish(time, joint_names, positions, velocities, std::vector<double>());
}

void ControllerCommandPublisher::publish(ros::Time time, std::vector<std::string> joint_names, std::vector<double> positions, std::vector<double> velocities, std::vector<double> accelerations)
{
    //TODO: implement

    publish(time, joint_names, positions, velocities, accelerations, std::vector<double>());
}

void ControllerCommandPublisher::publish(ros::Time time, std::vector<std::string> joint_names, std::vector<double> positions, std::vector<double> velocities, std::vector<double> accelerations, std::vector<double> max_currents)
{
    //TODO: implement

    bitbots_msgs::JointCommand message;
    std::vector<double> default_values(joint_names.size(), -1.0);
    
    message.header.stamp = time;

    message.joint_names = joint_names;
    message.positions = positions;
	message.velocities = velocities.empty() ? default_values : velocities;
	message.accelerations = accelerations.empty() ? default_values : accelerations;
	message.max_currents = max_currents.empty() ? default_values : max_currents;

    ros_publisher_controller_command_.publish(message);
}