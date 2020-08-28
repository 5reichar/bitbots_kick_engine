#include "ros_interface/throw_node.h"

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "utility/throw_utilities.h"
#include "utility/throw_stabilizer.h"

#include "ros_interface/publisher/system_publisher.h"

#include "parameter/throw_type_parameter_builder.h"

namespace bitbots_throw{
	ThrowNode::ThrowNode()
			:dynamic_reconfigure_server_engine_params_(ros::NodeHandle("/throw_engine_parameter"))
			,dynamic_reconfigure_server_throw_params_(ros::NodeHandle("/throw_parameter"))
			,tf2_ros_transform_listener_(tf2_ros_buffer_){
		set_default_parameter();
		load_parameter();
        init_ros_subscriptions();
		init_dynamic_reconfiguration();
		init_ik();
		SystemPublisher::publish_info("vDo16:20", "ThrowNode");
	}

	ThrowNode::~ThrowNode(){
	    delete arms_ik_;
	    delete legs_ik_;
	}

	void ThrowNode::set_default_parameter(){
		sp_node_parameter_ = ThrowNodeParameterBuilder::build_default();
		publisher_topics_.str_controller_command_topic_ = "/DynamixelController/command";
		publisher_topics_.str_odometry_topic_ = "/throw_odometry";
		publisher_topics_.str_debug_topic_ = "/throw_debug";
		publisher_topics_.str_debug_marker_topic_ = "/throw_debug_marker";
		publisher_topics_.str_support_topic_ = "/throw_support_foot_state";

        arms_ik_ = new ThrowIK("Arms"
                              ,{"LElbow", "LShoulderPitch", "LShoulderRoll", "RElbow", "RShoulderPitch", "RShoulderRoll"}
                              ,{0.7, -1.0, -0.4, -0.7, 1.0, 0.4});

        legs_ik_ = new ThrowIK("Legs"
                              ,{"LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch"}
                              ,{0.7, -1.0, -0.4, -0.7, 1.0, 0.4});
	}

	void ThrowNode::load_parameter(){
		ros_node_handle_.param<bool>("/simulation_active"
		                            ,sp_node_parameter_->simulation_active_
		                            ,false);
	}

	void ThrowNode::init_ros_subscriptions(){
		ros_subscriber_throw_ = ros_node_handle_.subscribe("/throw"
		                                                  ,1
		                                                  ,&ThrowNode::throw_callback
		                                                  ,this
		                                                  ,ros::TransportHints().tcpNoDelay());
	}

	void ThrowNode::init_dynamic_reconfiguration(){
		dynamic_reconfigure_server_engine_params_.setCallback([this](auto && PH1, auto && PH2){
		                                                        throw_engine_params_config_callback(PH1, PH2);
		                                                      });
		dynamic_reconfigure_server_throw_params_.setCallback([this](auto && PH1, auto && PH2){
		                                                        throw_params_config_callback(PH1, PH2);
		                                                      });
	}

	void ThrowNode::init_ik(){
		//load MoveIt! model
		robot_model_loader::RobotModelLoader robot_model_loader("/robot_description"
		                                                       ,false);
		robot_model_loader.loadKinematicsSolvers(std::make_shared<kinematics_plugin_loader::KinematicsPluginLoader>());
		moveit::core::RobotModelPtr kinematic_model;
		kinematic_model = robot_model_loader.getModel();

		if (!kinematic_model){
			SystemPublisher::publish_fatal("No robot model loaded, killing throw engine.");
			exit(1);
		}

		arms_ik_->init(kinematic_model);
		legs_ik_->init(kinematic_model);
	}

	void ThrowNode::throw_callback(const bitbots_throw::throw_action action){
		ThrowStabilizer stabilizer;
		throw_engine_.reset();
		ros::Rate loopRate(sp_node_parameter_->engine_frequency_);
		RosPublisherFacade publisher_facade(ros_node_handle_, sp_node_parameter_, publisher_topics_);

		publisher_facade.prepare_publisher_for_throw();
		auto throw_request = create_throw_request(action);
		throw_engine_.set_goals(throw_request);
        publisher_facade.publish_engine_debug(&throw_engine_, throw_request);

        int8_t percentage_done = throw_engine_.get_percent_done();
        int8_t movement_stage = throw_engine_.get_movement_stage();
        auto engine_update_dt = 1/sp_node_parameter_->engine_frequency_;
		while (ros::ok() && percentage_done < 100){
			auto response = throw_engine_.update(engine_update_dt);
			auto ik_goals = stabilizer.stabilize(response);
			bitbots_splines::JointGoals joint_goals;

			try{
				auto calc_goals = arms_ik_->calculate(std::move(ik_goals));
                joint_goals.first.insert(joint_goals.first.end(), calc_goals.first.begin(), calc_goals.first.end());
                joint_goals.second.insert(joint_goals.second.end(), calc_goals.second.begin(), calc_goals.second.end());

                calc_goals = legs_ik_->calculate(std::move(ik_goals));
                joint_goals.first.insert(joint_goals.first.end(), calc_goals.first.begin(), calc_goals.first.end());
                joint_goals.second.insert(joint_goals.second.end(), calc_goals.second.begin(), calc_goals.second.end());
			}
			catch(const std::runtime_error& e){
				SystemPublisher::publish_runtime_error(e);

				// maybe add some more diagnostic logic
				joint_goals = bitbots_splines::JointGoals();
			}
			
			publisher_facade.publish_throw(joint_goals);
			publisher_facade.publish_odometry();
			publisher_facade.publish_debug(response, percentage_done, movement_stage);

			ros::spinOnce();
			loopRate.sleep();
            percentage_done = throw_engine_.get_percent_done();
            movement_stage = throw_engine_.get_movement_stage();
		}
	}

	void ThrowNode::throw_engine_params_config_callback(bitbots_throw::throw_engine_paramsConfig & config , uint32_t level){
		sp_node_parameter_ = ThrowNodeParameterBuilder::build_from_dynamic_reconf(config, level);

        arms_ik_->set_bio_ik_timeout(sp_node_parameter_->bio_ik_time_);
        legs_ik_->set_bio_ik_timeout(sp_node_parameter_->bio_ik_time_);
		sp_engine_parameter_ = ThrowEngineParameterBuilder::build_from_dynamic_reconf(config, level);
		throw_engine_.set_engine_parameter(sp_engine_parameter_);
	}

	void ThrowNode::throw_params_config_callback(bitbots_throw::throw_paramsConfig & config , uint32_t level){
	    auto type_parameter = ThrowTypeParameterBuilder::build_from_dynamic_reconf(config, level);
		throw_engine_.set_throw_types(type_parameter);
	}

	ThrowRequest ThrowNode::create_throw_request(const bitbots_throw::throw_action action){
		ThrowRequest request{};

		request.ball_position_ = {action.ball_position.x, action.ball_position.y, action.ball_position.z };
		request.goal_position_ = {action.throw_target.x, action.throw_target.y, action.throw_target.z };

        // get current position of feet
        try{
            auto position = get_pose("r_sole").position;
            request.right_feet_position_ = {position.x, position.y, position.z};

            position = get_pose("l_sole").position;
            request.left_feet_position_ = {position.x, position.y, position.z};

            position = get_pose("r_wrist").position;
            request.right_hand_position_ = {position.x, position.y, position.z};

            position = get_pose("l_wrist").position;
            request.left_hand_position_ = {position.x, position.y, position.z};

            position = get_pose("head").position;
            request.head_position_ = {position.x, position.y, position.z};
        }
        catch(tf2::TransformException &e){
            SystemPublisher::publish_error(e.what(), "ThrowNode::create_throw_request()");

            request.head_position_ = {0.0, 0.0, sp_engine_parameter_->trunk_height_ + sp_engine_parameter_->head_height_/2};
            auto hand_height = (sp_engine_parameter_->trunk_height_ - sp_engine_parameter_->leg_length_);
            request.left_hand_position_ = {0.0, -0.15, hand_height};
            request.right_hand_position_ = {0.0, 0.15, hand_height};
            auto feet_height = -1 * sp_engine_parameter_->leg_length_;
            request.left_feet_position_ = {0.0, -0.1, feet_height};
            request.right_feet_position_ = {0.0, 0.1, feet_height};
        }

		return request;
	}

    geometry_msgs::Pose ThrowNode::get_pose(std::string const & frame_id
                                           ,double const & orientation
                                           ,ros::Time const & time
                                           ,std::string const & target_frame
                                           ,ros::Duration const & timeout){
        /* Construct zero-positions in their respective local frames */
	    geometry_msgs::PoseStamped origin;
        origin.header.frame_id = frame_id;
        origin.pose.orientation.w = orientation;
        origin.header.stamp = time;

        /* Transform both feet poses into the other foot's frame */
        geometry_msgs::PoseStamped transformed;
        tf2_ros_buffer_.transform(origin, transformed, target_frame, timeout);

        return transformed.pose;
	}
} //bitbots_throw

int main(int argc, char **argv){
	ros::init(argc, argv, "bitbots_throw");
	bitbots_throw::ThrowNode node;
	ros::spin();
}