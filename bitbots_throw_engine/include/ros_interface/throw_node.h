#ifndef THROW_NODE_H
#define THROW_NODE_H

#include "ros/ros.h"
#include "utility/throw_ik.h"
#include "engine/throw_engine.h"
#include "parameter/throw_node_parameter_builder.h"

#include <dynamic_reconfigure/server.h>
#include <bitbots_throw_engine/throw_action.h>
#include <bitbots_throw_engine/throw_paramsConfig.h>
#include <bitbots_throw_engine/throw_engine_paramsConfig.h>

class ThrowNode
{
public:
    ThrowNode();

private:
    // Initilization methodes
    void build_default_parameter();
    void load_parameter();
    void init_ros_subcribtions();
    void init_dynamic_reconfiguration();
    void init_ik();

    // Callback methodes
	void throw_callback(const bitbots_throw_engine::throw_action action);
    void throw_params_config_callback(bitbots_throw_engine::throw_paramsConfig & config , uint32_t level);
    void throw_engine_params_config_callback(bitbots_throw_engine::throw_engine_paramsConfig & config , uint32_t level);

    // Helper methodes
    ThrowRequest create_throw_request(const bitbots_throw_engine::throw_action action);

    // member variables
    ThrowIK ik_;
    ThrowEngine throw_engine_;
    std::shared_ptr<ThrowNodeParameter> sp_node_parameter_;

    ros::NodeHandle ros_node_handle_;
    ros::Subscriber ros_subsciber_throw_;
};

#endif