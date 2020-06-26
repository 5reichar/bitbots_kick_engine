#ifndef BITBOTS_THROW_THROW_NODE_H
#define BITBOTS_THROW_THROW_NODE_H

#include "ros/ros.h"
#include "utility/throw_ik.h"
#include "engine/throw_engine.h"
#include "parameter/throw_node_parameter_builder.h"

#include <dynamic_reconfigure/server.h>
#include <bitbots_throw/throw_action.h>
#include <bitbots_throw/throw_paramsConfig.h>
#include <bitbots_throw/throw_engine_paramsConfig.h>
#include "ros_interface/publisher/ros_publisher_facade.h"

namespace bitbots_throw{
    class ThrowNode
    {
    public:
        ThrowNode();
        ~ThrowNode();

    private:
        // Initialization methods
        void set_default_parameter();
        void load_parameter();
        void init_ros_subscriptions();
        void init_dynamic_reconfiguration();
        void init_ik();

        // Callback methods
        void throw_callback(bitbots_throw::throw_action action);
        void throw_params_config_callback(bitbots_throw::throw_paramsConfig & config , uint32_t level);
        void throw_engine_params_config_callback(bitbots_throw::throw_engine_paramsConfig & config , uint32_t level);

        // Helper methods
        static ThrowRequest create_throw_request(bitbots_throw::throw_action action);

        // member variables
        ThrowIK * arms_ik_;
        ThrowIK * legs_ik_;

        ThrowEngine throw_engine_;
        std::shared_ptr<ThrowNodeParameter> sp_node_parameter_;
        RosPublisherFacade::RosPublisherTopics publisher_topics_;

        ros::NodeHandle ros_node_handle_;
        ros::Subscriber ros_subscriber_throw_;
        dynamic_reconfigure::Server<bitbots_throw::throw_paramsConfig> dynamic_reconfigure_server_throw_params_;
        dynamic_reconfigure::Server<bitbots_throw::throw_engine_paramsConfig> dynamic_reconfigure_server_engine_params_;
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_NODE_H