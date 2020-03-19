#ifndef THROW_NODE_H
#define THROW_NODE_H

#include "ros/ros.h"
#include "ros_interface/throw_ros.h"
#include <bitbots_throw_engine/throw_action.h>
#include <humanoid_league_msgs/RobotControlState.h>

class ThrowNode
{
    // TODO cleanup
public:
    ThrowNode(/* args */);
	~ThrowNode();

private:
    void init_ros_publisher();
    void init_ros_subcribtions();

	void throw_callback(const bitbots_throw_engine::throw_action action);
	void robot_state_callback(const humanoid_league_msgs::RobotControlState msg);

    std::unique_ptr<ThrowRos> throw_ros_;

    ros::NodeHandle ros_node_handle_;

    ros::Publisher ros_publisher_controller_command_;
    ros::Publisher ros_publisher_odometry_;
    ros::Publisher ros_publisher_support_;

    ros::Publisher ros_publisher_debug_;
    ros::Publisher ros_publisher_debug_marker_;

    ros::Subscriber ros_subsciber_throw_;
    ros::Subscriber ros_subsciber_robot_state_;
};

#endif