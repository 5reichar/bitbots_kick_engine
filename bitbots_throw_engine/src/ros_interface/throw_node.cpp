#include "ros_interface/throw_node.h"
#include <nav_msgs/Odometry.h>
#include <bitbots_msgs/JointCommand.h>
#include <std_msgs/Char.h>

ThrowNode::ThrowNode()
{
	// TODO testing
	// TODO cleanup

	init_ros_publisher();
	init_ros_subcribtions();
}

ThrowNode::~ThrowNode()
{
	// TODO testing
	// TODO cleanup
}

void ThrowNode::init_ros_publisher()
{
	// TODO testing
	// TODO cleanup

	ros_publisher_controller_command_ = ros_node_handle_.advertise<bitbots_msgs::JointCommand>("throw_motor_goals", 1);
	ros_publisher_odometry_ = ros_node_handle_.advertise<nav_msgs::Odometry>("throw_odometry", 1);
	ros_publisher_support_ = ros_node_handle_.advertise<std_msgs::Char>("throw_support_state", 1);
}

void ThrowNode::init_ros_subcribtions()
{
	// TODO testing
	// TODO cleanup

	ros_subsciber_throw_ = ros_node_handle_.subscribe("throw", 1, &ThrowNode::throw_callback, this, ros::TransportHints().tcpNoDelay());
}

void ThrowNode::throw_callback(const bitbots_throw_engine::throw_action action)
{
	// TODO testing
	// TODO cleanup

	struct3d ball = {action.ball_position.x, action.ball_position.y, action.ball_position.z };
	struct3d goal = {action.throw_target.x, action.throw_target.y, action.throw_target.z };

	throw_ros_->run_throw(ball, goal);
}