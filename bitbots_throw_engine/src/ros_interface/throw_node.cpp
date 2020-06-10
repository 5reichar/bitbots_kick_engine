#include "ros_interface/throw_node.h"

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "utility/throw_utilities.h"
#include "utility/throw_stabilizer.h"

#include "ros_interface/publisher/system_publisher.h"

#include "parameter/throw_engine_parameter_builder.h"
#include "parameter/throw_type_parameter_builder.h"

namespace bitbots_throw{
	ThrowNode::ThrowNode()
			: dynamic_reconfigure_server_engine_params_(ros::NodeHandle("/throw_engine_parameter"))
			, dynamic_reconfigure_server_throw_params_(ros::NodeHandle("/throw_parameter")){
		set_default_parameter();
		load_parameter();
		init_ros_subcribtions();
		init_dynamic_reconfiguration();
		init_ik();
	}

	void ThrowNode::set_default_parameter(){
		sp_node_parameter_ = ThrowNodeParameterBuilder::build_default();
		publisher_topics_.str_controller_command_topic_ = "/throwing_motor_goals";
		publisher_topics_.str_odometry_topic_ = "/throw_odometry";
		publisher_topics_.str_debug_topic_ = "/throw_debug";
		publisher_topics_.str_engine_debug_topic_ = "/throw_engine_debug";
		publisher_topics_.str_debug_marker_topic_ = "/throw_debug_marker";
		publisher_topics_.str_support_topic_ = "/throw_support_foot_state";
	}

	void ThrowNode::load_parameter(){
		ros_node_handle_.param<bool>("/simulation_active", sp_node_parameter_->simulation_active_, false);
	}

	void ThrowNode::init_ros_subcribtions(){
		ros_subsciber_throw_ = ros_node_handle_.subscribe("/throw", 1, &ThrowNode::throw_callback, this, ros::TransportHints().tcpNoDelay());
	}

	void ThrowNode::init_dynamic_reconfiguration(){
		dynamic_reconfigure_server_engine_params_.setCallback(boost::bind(&ThrowNode::throw_engine_params_config_callback, this, _1, _2));
		dynamic_reconfigure_server_throw_params_.setCallback(boost::bind(&ThrowNode::throw_params_config_callback, this, _1, _2));
	}

	void ThrowNode::init_ik(){
		//load MoveIt! model
		robot_model_loader::RobotModelLoader robot_model_loader("/robot_description", false);
		robot_model_loader.loadKinematicsSolvers(std::make_shared<kinematics_plugin_loader::KinematicsPluginLoader>());
		moveit::core::RobotModelPtr kinematic_model;
		kinematic_model = robot_model_loader.getModel();

		if (!kinematic_model){
			SystemPublisher::publish_fatal("No robot model loaded, killing throw engine.");
			exit(1);
		}

		ik_.init(kinematic_model);
	}

	void ThrowNode::throw_callback(const bitbots_throw::throw_action action){
		ThrowStabilizer stabilizer;
		throw_engine_.reset();
		ros::Rate loopRate(sp_node_parameter_->engine_frequency_);
		RosPublisherFacade publisher_facade(ros_node_handle_, sp_node_parameter_, publisher_topics_);

		publisher_facade.prepare_publisher_for_throw();
		throw_engine_.set_goals(create_throw_request(action));

		while (ros::ok() && throw_engine_.get_percent_done() < 100){
			auto response = throw_engine_.update(1/sp_node_parameter_->engine_frequency_);
			auto ik_goals = stabilizer.stabilize(response);
			bitbots_splines::JointGoals joint_goals;

			try{
				joint_goals = ik_.calculate(std::move(ik_goals));
			}
			catch(const std::runtime_error& e){
				SystemPublisher::publish_runtime_error(e);

				// maybe add some more diagnostic logic
				joint_goals = bitbots_splines::JointGoals();
			}
			
			publisher_facade.publish_throw(joint_goals);
			publisher_facade.publish_odometry();
			publisher_facade.publish_debug(response, joint_goals);

			ros::spinOnce();
			loopRate.sleep();
		}
	}

	void ThrowNode::throw_engine_params_config_callback(bitbots_throw::throw_engine_paramsConfig & config , uint32_t level){
		sp_node_parameter_ = ThrowNodeParameterBuilder::build_from_dynamic_reconf(config, level);

		ik_.set_bio_ik_timeout(sp_node_parameter_->bio_ik_time_);
		auto t = ThrowEngineParameterBuilder::build_from_dynamic_reconf(config, level);
		throw_engine_.set_engine_parameter(t);
	}

	void ThrowNode::throw_params_config_callback(bitbots_throw::throw_paramsConfig & config , uint32_t level){
		throw_engine_.set_throw_types(ThrowTypeParameterBuilder::build_from_dynamic_reconf(config, level));
	}

	ThrowRequest ThrowNode::create_throw_request(const bitbots_throw::throw_action action){
		ThrowRequest request;
		request.ball_position_ = {action.ball_position.x, action.ball_position.y, action.ball_position.z };
		request.goal_position_ = {action.throw_target.x, action.throw_target.y, action.throw_target.z };
		return request;
	}
} //bitbots_throw

int main(int argc, char **argv){
	ros::init(argc, argv, "bitbots_throw");
	bitbots_throw::ThrowNode node;
	ros::spin();
}