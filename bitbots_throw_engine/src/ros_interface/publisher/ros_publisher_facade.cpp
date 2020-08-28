#include "ros_interface/publisher/ros_publisher_facade.h"

namespace bitbots_throw{
	RosPublisherFacade::RosPublisherFacade(ros::NodeHandle & ros_node_handle
	                                      ,std::shared_ptr<ThrowNodeParameter> parameter
	                                      ,RosPublisherTopics const &topics)
		: sp_node_parameter_(parameter)
		, sp_controller_command_publisher_(new ControllerCommandPublisher(ros_node_handle
		                                                                    ,topics.str_controller_command_topic_))
		, sp_odometry_publisher_(new OdometryPublisher(ros_node_handle, topics.str_odometry_topic_))
		, sp_support_publisher_(new SupportPublisher(ros_node_handle, topics.str_support_topic_))
		, sp_debug_publisher_(new DebugPublisher(ros_node_handle
		                                           ,topics.str_debug_topic_
		                                           ,topics.str_debug_marker_topic_)){
	}

	void RosPublisherFacade::prepare_publisher_for_throw(){
		sp_odometry_publisher_->reset_counter();
	}

	void RosPublisherFacade::publish_throw(bitbots_splines::JointGoals & joint_goals){
		auto time = ros::Time::now();

		sp_controller_command_publisher_->publish(time, joint_goals);
		sp_support_publisher_->publish();
	}

	void RosPublisherFacade::publish_odometry(){
		sp_odometry_publisher_->publish(sp_node_parameter_->odom_publish_factor_);
	}

    void RosPublisherFacade::publish_engine_debug(ThrowEngine const * engine, ThrowRequest const & request) const{
	    if(sp_node_parameter_->debug_active_){
	        std::stringstream stream;

            stream << "Throw Request,Ball,,,,Goal,,,,Head,,,,Left Hand,,,,Right Hand,,,,Left Foot,,,,Right Foot,," << std::endl;
            stream << "Position,x,y,z,,x,y,z,,x,y,z,,x,y,z,,x,y,z,,x,y,z,,x,y,z" << std::endl;
            stream << "values,";
            stream << request.ball_position_.x_ << "," << request.ball_position_.y_ << "," << request.ball_position_.z_ << ",,";
            stream << request.goal_position_.x_ << "," << request.goal_position_.y_ << "," << request.goal_position_.z_ << ",,";
            stream << request.head_position_.x_ << "," << request.head_position_.y_ << "," << request.head_position_.z_ << ",,";
            stream << request.left_hand_position_.x_ << "," << request.left_hand_position_.y_ << "," << request.left_hand_position_.z_ << ",,";
            stream << request.right_hand_position_.x_ << "," << request.right_hand_position_.y_ << "," << request.right_hand_position_.z_ << ",,";
            stream << request.left_feet_position_.x_ << "," << request.left_feet_position_.y_ << "," << request.left_feet_position_.z_ << ",,";
            stream << request.right_feet_position_.x_ << "," << request.right_feet_position_.y_ << "," << request.right_feet_position_.z_ << std::endl;
	        stream << engine->get_throw_points_as_string();

	        sp_debug_publisher_->print_throw_points(stream.str());
	    }
    }

	void RosPublisherFacade::publish_debug(ThrowResponse const & response, int8_t const & percentage_done, int8_t const & movement_stage){
		if(sp_node_parameter_->debug_active_){
			sp_debug_publisher_->publish_ik_debug(response, percentage_done, movement_stage);
		}
	}
} //bitbots_throw