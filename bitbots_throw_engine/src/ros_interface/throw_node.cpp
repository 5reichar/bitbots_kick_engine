#include "ros_interface/throw_node.h"
#include <dynamic_reconfigure/server.h>
#include "utility/throw_utilities.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_base/kinematics_base.h>

ThrowNode::ThrowNode()
{
	//TODO: testing
	//TODO: cleanup

	ros_node_handle_.param<bool>("/simulation_active", sp_node_parameter_->simulation_active_, false);

	up_publisher_facade_.reset(new RosPublisherFacade(ros_node_handle_, sp_node_parameter_));
	init_ros_subcribtions();
	init_dynamic_reconfiguration();

	//load MoveIt! model
	robot_model_loader::RobotModelLoader robot_model_loader("/robot_description", false);
	robot_model_loader.loadKinematicsSolvers(std::make_shared<kinematics_plugin_loader::KinematicsPluginLoader>());
	 robot_model::RobotModelPtr kinematic_model(robot_model_loader.getModel());
	if (!kinematic_model)
	{
		ROS_FATAL("No robot model loaded, killing quintic walk.");
		exit(1);
	}
	up_throw_ik->init(kinematic_model);
}

void ThrowNode::init_ros_subcribtions()
{
	//TODO: testing
	//TODO: cleanup

	ros_subsciber_throw_ = ros_node_handle_.subscribe("throw", 1, &ThrowNode::throw_callback, this, ros::TransportHints().tcpNoDelay());
}

void ThrowNode::init_dynamic_reconfiguration()
{
	//TODO: testing
	//TODO: cleanup

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
	//TODO: testing
	//TODO: cleanup

	up_publisher_facade_->prepare_publisher_for_throw();

	ThrowRequest request;
	request.ball_position_ = {action.ball_position.x, action.ball_position.y, action.ball_position.z };
	request.goal_position_ = {action.throw_target.x, action.throw_target.y, action.throw_target.z };
	up_throw_engine_->set_goals(request);

	ros::Rate loopRate(sp_node_parameter_->engine_frequency_);

	while (ros::ok())
	{
		auto response = up_throw_engine_->update(1/sp_node_parameter_->engine_frequency_);
		auto ik_goals = up_throw_stabilizer->stabilize(response);
		auto joint_goals = up_throw_ik->calculate(std::move(ik_goals));
		
		up_publisher_facade_->publish_throw(joint_goals);
		up_publisher_facade_->publish_odometry();
		up_publisher_facade_->publish_debug();

		ros::spinOnce();
		loopRate.sleep();
	}

}

void ThrowNode::throw_engine_params_config_callback(bitbots_throw_engine::throw_engine_paramsConfig & config , uint32_t level)
{
	sp_node_parameter_ = ThrowNodeParameterBuilder::build_from_dynamic_reconf(config, level);

	up_throw_ik->set_bio_ik_timeout(sp_node_parameter_->bio_ik_time_);
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