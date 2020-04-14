#include "ros_interface/throw_node.h"
#include <dynamic_reconfigure/server.h>

ThrowNode::ThrowNode()
{
	// TODO testing
	// TODO cleanup

	ros_node_handle_.param<bool>("/simulation_active", sp_node_parameter_->simulation_active_, false);

	up_publisher_facade_.reset(new RosPublisherFacade(ros_node_handle_, sp_node_parameter_));
	init_ros_subcribtions();
	init_dynamic_reconfiguration();
}

void ThrowNode::init_ros_subcribtions()
{
	// TODO testing
	// TODO cleanup

	ros_subsciber_throw_ = ros_node_handle_.subscribe("throw", 1, &ThrowNode::throw_callback, this, ros::TransportHints().tcpNoDelay());
}

void ThrowNode::init_dynamic_reconfiguration()
{
	// TODO testing
	// TODO cleanup

	dynamic_reconfigure::Server<bitbots_throw_engine::throw_engine_paramsConfig> engine_param_server;
	dynamic_reconfigure::Server<bitbots_throw_engine::throw_engine_paramsConfig>::CallbackType engine_params;
	engine_params = boost::bind(&ThrowNode::throw_engine_params_config_callback, this, _1, _2);
	engine_param_server.setCallback(engine_params);

	dynamic_reconfigure::Server<bitbots_throw_engine::throw_paramsConfig> throw_param_server;
	dynamic_reconfigure::Server<bitbots_throw_engine::throw_paramsConfig>::CallbackType throw_params;
	throw_params = boost::bind(&ThrowNode::throw_params_config_callback, this, _1, _2);
	throw_param_server.setCallback(throw_params);
}

void ThrowNode::throw_callback(const bitbots_throw_engine::throw_action action)
{
	// TODO testing
	// TODO cleanup

	up_publisher_facade_->prepare_publisher_for_throw();

	struct3d ball_position = {action.ball_position.x, action.ball_position.y, action.ball_position.z };
	struct3d goal_position = {action.throw_target.x, action.throw_target.y, action.throw_target.z };

	ros::Rate loopRate(sp_node_parameter_->engine_frequency_);

	while (ros::ok())
	{
		if (up_throw_engine_->throw_ball(ball_position, goal_position))
		{
			up_publisher_facade_->publish_throw();
		}

		up_publisher_facade_->publish_odometry();

		ros::spinOnce();
		loopRate.sleep();
	}

}

void ThrowNode::robot_state_callback(const humanoid_league_msgs::RobotControlState msg)
{

}

void ThrowNode::throw_engine_params_config_callback(bitbots_throw_engine::throw_engine_paramsConfig & config , uint32_t level)
{
	sp_node_parameter_ = ThrowNodeParameterBuilder::build_from_dynamic_reconf(config, level);
	up_throw_engine_->set_engine_parameter(ThrowEngineParameterBuilder::build_from_dynamic_reconf(config, level));
}

void ThrowNode::throw_params_config_callback(bitbots_throw_engine::throw_paramsConfig & config , uint32_t level)
{
	up_throw_engine_->set_throw_types(ThrowTypeParameterBuilder::build_from_dynamic_reconf(config, level));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bitbots_throw_engine");

	return 0;
}