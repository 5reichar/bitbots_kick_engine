#ifndef THROW_NODE_H
#define THROW_NODE_H

#include "ros/ros.h"
#include "engine/throw_engine.h"
#include <bitbots_throw_engine/throw_action.h>
#include <humanoid_league_msgs/RobotControlState.h>
#include "ros_interface/publisher/ros_publisher_facade.h"
#include "parameter/throw_engine_parameter_builder.h"
#include "parameter/throw_node_parameter_builder.h"
#include "parameter/throw_type_parameter_builder.h"

class ThrowNode
{
    // TODO cleanup
public:
    ThrowNode(/* args */);

private:
    void init_ros_subcribtions();
    void init_dynamic_reconfiguration();

	void throw_callback(const bitbots_throw_engine::throw_action action);
	void robot_state_callback(const humanoid_league_msgs::RobotControlState msg);

    void throw_engine_params_config_callback(bitbots_throw_engine::throw_engine_paramsConfig & config , uint32_t level);
    void throw_params_config_callback(bitbots_throw_engine::throw_paramsConfig & config , uint32_t level);

    std::unique_ptr<ThrowEngine> up_throw_engine_;
    std::shared_ptr<ThrowNodeParameter> sp_node_parameter_;

    ros::NodeHandle ros_node_handle_;
    std::unique_ptr<RosPublisherFacade> up_publisher_facade_;

    ros::Subscriber ros_subsciber_throw_;
    ros::Subscriber ros_subsciber_robot_state_;
};

#endif