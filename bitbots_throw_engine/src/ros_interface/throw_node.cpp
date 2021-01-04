#include "ros_interface/throw_node.h"
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include "utility/throw_utilities.h"
#include "utility/throw_stabilizer.h"
#include "ros_interface/publisher/system_publisher.h"
#include "ros_interface/ros_joint_and_topic_names.h"
#include "parameter/throw_type_parameter_builder.h"

namespace bitbots_throw{
	ThrowNode::ThrowNode()
			:dynamic_reconfigure_server_engine_params_(ros::NodeHandle("/throw_engine_parameter"))
			,dynamic_reconfigure_server_throw_params_(ros::NodeHandle("/throw_parameter"))
			,tf2_ros_transform_listener_(tf2_ros_buffer_){
        init_dynamic_reconfiguration();
        load_parameter();
        init_ros_subscriptions();
		init_ik();
		SystemPublisher::publish_info("v0.20201120-2", "ThrowNode");
	}

	void ThrowNode::set_default_parameter(){
        ThrowVisualizer::ThrowVisualizerParams parameter;
        parameter.left_hand_frame_ = RosJointAndTopicNames::get_joint_base_link();
        parameter.right_hand_frame_ = RosJointAndTopicNames::get_joint_base_link();
        parameter.left_foot_frame_ = RosJointAndTopicNames::get_joint_base_link();
        parameter.right_foot_frame_ = RosJointAndTopicNames::get_joint_base_link();

        sp_publisher_facade_.reset(new RosPublisherFacade(ros_node_handle_, sp_engine_parameter_, parameter, sp_debug_parameter_));
	}

	void ThrowNode::load_parameter(){
		ros_node_handle_.param<bool>("/simulation_active"
		                            ,sp_engine_parameter_->simulation_active_
		                            ,false);
	}

	void ThrowNode::init_ros_subscriptions(){
		ros_subscriber_throw_ = ros_node_handle_.subscribe(RosJointAndTopicNames::get_topic_throw()
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

		stabilizer_and_ik_.set_kinematic_model(kinematic_model);
	}

	void ThrowNode::throw_callback(const bitbots_throw::throw_action action){
		throw_engine_.reset();
		ros::Rate loopRate(sp_engine_parameter_->engine_frequency_);

        sp_publisher_facade_->prepare_publisher_for_throw();
		auto throw_request = create_throw_request(action);
		throw_engine_.set_goals(throw_request);
		sp_publisher_facade_->visualize_engine(&throw_engine_);

        int8_t percentage_done = throw_engine_.get_percent_done();
        int8_t movement_stage = throw_engine_.get_movement_stage();
        auto engine_update_dt = 1 / sp_engine_parameter_->engine_frequency_;
        std::vector<ThrowResponse> vec_responses;
		while (ros::ok() && percentage_done < 100){
			auto response = throw_engine_.update(engine_update_dt);
			vec_responses.emplace_back(response);
			auto joint_goals = stabilizer_and_ik_.calculate_joint_goals(response);
			
			sp_publisher_facade_->publish_throw(joint_goals);
			sp_publisher_facade_->publish_odometry();
			sp_publisher_facade_->publish_debug(response, percentage_done, movement_stage);

			ros::spinOnce();
			loopRate.sleep();
            percentage_done = throw_engine_.get_percent_done();
            movement_stage = throw_engine_.get_movement_stage();
		}
        sp_publisher_facade_->publish_engine_debug(&throw_engine_, throw_request, vec_responses);
	}

	void ThrowNode::throw_engine_params_config_callback(throw_engine_paramsConfig & config , uint32_t level){
        sp_debug_parameter_.reset(new ThrowDebugParameter(config, level));
        sp_engine_parameter_.reset(new ThrowEngineParameter(config, level));
        sp_robot_and_world_parameter_.reset(new RobotAndWorldParameter(config, level));

		if(!sp_publisher_facade_ ){
		    set_default_parameter();
		}

        sp_publisher_facade_->set_parameter(sp_engine_parameter_, sp_debug_parameter_);

		stabilizer_and_ik_.set_debug_parameter(sp_debug_parameter_);
		stabilizer_and_ik_.set_bio_ik_timeout(sp_engine_parameter_->bio_ik_time_);

        throw_engine_.set_engine_parameter(sp_robot_and_world_parameter_);
	}

	void ThrowNode::throw_params_config_callback(throw_paramsConfig & config , uint32_t level){
	    auto type_parameter = ThrowTypeParameterBuilder::build_from_dynamic_reconf(config, level, sp_robot_and_world_parameter_);
		throw_engine_.set_throw_types(type_parameter);
	}

	ThrowRequest ThrowNode::create_throw_request(const throw_action action){
		ThrowRequest request{};
		bool set_default_values = false;

		request.ball_position_ = {action.ball_position.x, action.ball_position.y, action.ball_position.z };
		request.goal_position_ = {action.throw_target.x, action.throw_target.y, action.throw_target.z };

        if(sp_engine_parameter_->use_default_start_value_){
            set_default_values = true;
        }else{
            // get current position of feet
            try{
                auto pose = get_pose(RosJointAndTopicNames::get_joint_r_sole());
                request.joint_start_position_[RobotJoints::right_foot] = {pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
                                                pose.orientation.y, pose.orientation.z};

                pose = get_pose(RosJointAndTopicNames::get_joint_l_sole());
                request.joint_start_position_[RobotJoints::left_foot] = {pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
                                               pose.orientation.y, pose.orientation.z};

                pose = get_pose(RosJointAndTopicNames::get_joint_r_wrist());
                request.joint_start_position_[RobotJoints::right_hand] = {pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
                                                pose.orientation.y, pose.orientation.z};

                pose = get_pose(RosJointAndTopicNames::get_joint_l_wrist());
                request.joint_start_position_[RobotJoints::left_hand] = {pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
                                               pose.orientation.y, pose.orientation.z};

                pose = get_pose(RosJointAndTopicNames::get_joint_head());
                request.joint_start_position_[RobotJoints::head] = {pose.position.x, pose.position.y, pose.position.z, pose.orientation.x,
                                          pose.orientation.y, pose.orientation.z};

                if(sp_debug_parameter_->debug_active_){
                    SystemPublisher::publish_info("used robot position", "ThrowNode::create_throw_request");
                }
            }
            catch(tf2::TransformException & e){
                SystemPublisher::publish_error(e.what(), "ThrowNode::create_throw_request()");
                set_default_values = true;
            }
        }

        if(set_default_values){
            if(sp_debug_parameter_->debug_active_){
                SystemPublisher::publish_info("used default position", "ThrowNode::create_throw_request");
            }

            auto hand_height = sp_robot_and_world_parameter_->trunk_height_;
            request.joint_start_position_[RobotJoints::head] = {0.0, 0.0, hand_height, 0.0, 0.0, 0.0};
            hand_height -= sp_robot_and_world_parameter_->arm_length_;
            request.joint_start_position_[RobotJoints::left_hand] = {0.0, sp_robot_and_world_parameter_->arm_distance_, hand_height, 0.0, 0.0, 0.0};
            request.joint_start_position_[RobotJoints::right_hand] = {0.0, -1 * sp_robot_and_world_parameter_->arm_distance_, hand_height, 0.0, 0.0, 0.0};
            auto feet_height = -1 * sp_robot_and_world_parameter_->leg_length_;
            request.joint_start_position_[RobotJoints::left_foot] = {0.0, sp_robot_and_world_parameter_->leg_distance_, feet_height, 0.0, 0.0, 0.0};
            request.joint_start_position_[RobotJoints::right_foot] = {0.0, -1 * sp_robot_and_world_parameter_->leg_distance_, feet_height, 0.0, 0.0, 0.0};
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