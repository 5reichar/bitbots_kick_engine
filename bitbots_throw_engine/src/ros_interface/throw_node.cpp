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
		SystemPublisher::publish_info("vDi-15:50", "ThrowNode");
	}

	void ThrowNode::set_default_parameter(){
		sp_node_parameter_ = ThrowNodeParameterBuilder::build_default();

        RosPublisherFacade::RosPublisherTopics publisher_topics;
        publisher_topics.str_controller_command_topic_ = "/DynamixelController/command";
        publisher_topics.str_odometry_topic_ = "/throw_odometry";
        publisher_topics.str_debug_topic_ = "/throw_debug";
        publisher_topics.str_debug_marker_topic_ = "/throw_debug_marker";
        publisher_topics.str_support_topic_ = "/throw_support_foot_state";
        publisher_topics.str_debug_visualization_base_topic_ = "/throw_debug";

        ThrowVisualizer::ThrowVisualizerParams parameter;
        parameter.smoothness_ = sp_node_parameter_->visualization_smoothness_;
        parameter.left_hand_frame = "base_link";
        parameter.right_hand_frame = "base_link";
        parameter.left_foot_frame = "base_link";
        parameter.right_foot_frame = "base_link";

        sp_publisher_facade_.reset(new RosPublisherFacade(ros_node_handle_, sp_node_parameter_, publisher_topics, parameter));
        sp_ik_left_arm_.reset(new ThrowIK("LeftArm", {"LElbow", "LShoulderPitch", "LShoulderRoll"}, {0.0, 0.0, 0.0}));
        sp_ik_right_arm_.reset(new ThrowIK("RightArm", {"RElbow", "RShoulderPitch", "RShoulderRoll"}, {0.0, 0.0, 0.0}));
        sp_ik_left_foot_.reset(new ThrowIK("LeftLeg", {"LHipPitch", "LKnee", "LAnklePitch"}, {0.7, 1.0, -0.4}));
        sp_ik_right_foot_.reset(new ThrowIK("RightLeg", {"RHipPitch", "RKnee", "RAnklePitch"}, {-0.7, -1.0, 0.4}));
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

        sp_ik_left_arm_->init(kinematic_model);
        sp_ik_right_arm_->init(kinematic_model);
        sp_ik_left_foot_->init(kinematic_model);
        sp_ik_right_foot_->init(kinematic_model);
	}

	void ThrowNode::throw_callback(const bitbots_throw::throw_action action){
		throw_engine_.reset();
		ros::Rate loopRate(sp_node_parameter_->engine_frequency_);

        sp_publisher_facade_->prepare_publisher_for_throw();
		auto throw_request = create_throw_request(action);
		throw_engine_.set_goals(throw_request);
        sp_publisher_facade_->publish_engine_debug(&throw_engine_, throw_request);

        int8_t percentage_done = throw_engine_.get_percent_done();
        int8_t movement_stage = throw_engine_.get_movement_stage();
        auto engine_update_dt = 1/sp_node_parameter_->engine_frequency_;
		while (ros::ok() && percentage_done < 100){
			auto response = throw_engine_.update(engine_update_dt);
			bitbots_splines::JointGoals joint_goals;

			try{
				joint_goals = calculate_joint_goals(response);
			}
			catch(const std::runtime_error& e){
				SystemPublisher::publish_runtime_error(e);

				// maybe add some more diagnostic logic
				joint_goals = bitbots_splines::JointGoals();
			}
			
			sp_publisher_facade_->publish_throw(joint_goals);
			sp_publisher_facade_->publish_odometry();
			sp_publisher_facade_->publish_debug(response, percentage_done, movement_stage);

			ros::spinOnce();
			loopRate.sleep();
            percentage_done = throw_engine_.get_percent_done();
            movement_stage = throw_engine_.get_movement_stage();
		}
	}

	void ThrowNode::throw_engine_params_config_callback(bitbots_throw::throw_engine_paramsConfig & config , uint32_t level){
		sp_node_parameter_ = ThrowNodeParameterBuilder::build_from_dynamic_reconf(config, level);
        sp_publisher_facade_->update_node_parameter(sp_node_parameter_);
        sp_ik_left_arm_->set_bio_ik_timeout(sp_node_parameter_->bio_ik_time_);
        sp_ik_right_arm_->set_bio_ik_timeout(sp_node_parameter_->bio_ik_time_);
        sp_ik_left_foot_->set_bio_ik_timeout(sp_node_parameter_->bio_ik_time_);
        sp_ik_right_foot_->set_bio_ik_timeout(sp_node_parameter_->bio_ik_time_);

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

            position = get_pose("base_link").position;
            request.trunk_position_ = {position.x, position.y, position.z};
        }
        catch(tf2::TransformException &e){
            SystemPublisher::publish_error(e.what(), "ThrowNode::create_throw_request()");

            auto hand_height = sp_engine_parameter_->trunk_height_;
            request.head_position_ = {0.0, 0.0, hand_height};
            request.trunk_position_ = {0.0, 0.0, 0.0};
            hand_height -= sp_engine_parameter_->arm_length_;
            request.left_hand_position_ = {0.0, -0.15, hand_height};
            request.right_hand_position_ = {0.0, 0.15, hand_height};
            auto feet_height = -1 * sp_engine_parameter_->leg_length_;
            request.left_feet_position_ = {0.0, -0.1, feet_height};
            request.right_feet_position_ = {0.0, 0.1, feet_height};
        }

		return request;
	}

    bitbots_splines::JointGoals ThrowNode::calculate_joint_goals(ThrowResponse const & response){
        bitbots_splines::JointGoals joint_goals;

        std::vector<ThrowStabilizerData> data = {{response.support_foot_to_left_hand_, "l_wrist", "l_sole", 1}};
        calculate_goal(sp_ik_left_arm_, joint_goals, data);

        data = {{response.support_foot_to_right_hand_, "r_wrist", "l_sole", 1}};
        calculate_goal(sp_ik_right_arm_, joint_goals, data);

        data = {{response.support_foot_to_left_foot_, "l_sole", "l_sole", 1}};
        calculate_goal(sp_ik_left_foot_, joint_goals, data);

        data = {{response.support_foot_to_right_foot_, "r_sole", "l_sole", 1}};
        calculate_goal(sp_ik_right_foot_, joint_goals, data);

        return joint_goals;
    }

    void ThrowNode::calculate_goal(std::shared_ptr <ThrowIK> & ik, bitbots_splines::JointGoals & joint_goals, std::vector <ThrowStabilizerData> & data){
        ThrowStabilizer stabilizer;
        auto ik_goals = stabilizer.stabilize(data);
        auto calc_goals = ik->calculate(std::move(ik_goals));
        joint_goals.first.insert(joint_goals.first.end(), calc_goals.first.begin(), calc_goals.first.end());
        joint_goals.second.insert(joint_goals.second.end(), calc_goals.second.begin(), calc_goals.second.end());
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