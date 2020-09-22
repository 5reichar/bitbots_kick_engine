#include "ros_interface/publisher/ros_publisher_facade.h"
#include "ros_interface/publisher/system_publisher.h"

namespace bitbots_throw{
	RosPublisherFacade::RosPublisherFacade(ros::NodeHandle & ros_node_handle
	                                      ,std::shared_ptr<ThrowNodeParameter> & parameter
	                                      ,RosPublisherTopics const &topics
	                                      ,ThrowVisualizer::ThrowVisualizerParams const & visualization_parameter)
		: sp_node_parameter_(parameter)
		, sp_visualizer_(new ThrowVisualizer(topics.str_debug_visualization_base_topic_, visualization_parameter, sp_node_parameter_))
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

    void RosPublisherFacade::update_node_parameter(std::shared_ptr<ThrowNodeParameter> & parameter){
        sp_node_parameter_ = parameter;
        sp_visualizer_->update_node_parameter(parameter);
    }

	void RosPublisherFacade::publish_throw(bitbots_splines::JointGoals & joint_goals){
		auto time = ros::Time::now();

		sp_controller_command_publisher_->publish(time, joint_goals);
		sp_support_publisher_->publish();
	}

	void RosPublisherFacade::publish_odometry(){
		sp_odometry_publisher_->publish(sp_node_parameter_->odom_publish_factor_);
	}

    void RosPublisherFacade::publish_engine_debug(ThrowEngine * engine, ThrowRequest const & request, std::vector<ThrowResponse> & responses){
	    if(sp_node_parameter_->debug_active_){
	        SystemPublisher::publish_info("Printing Engine data", "RosPublisherFacade");
	        std::stringstream stream;

            stream << "Throw Request,Ball,,,,Goal,,,,Head,,,,Left Hand,,,,Right Hand,,,,Left Foot,,,,Right Foot,," << std::endl;
            stream << "Position,x,y,z,,x,y,z,,x,y,z,roll,pitch,yaw,,x,y,z,roll,pitch,yaw,,x,y,z,roll,pitch,yaw,,x,y,z,roll,pitch,yaw,,x,y,z,roll,pitch,yaw" << std::endl;
            stream << "values,";
            stream << request.ball_position_.x_ << "," << request.ball_position_.y_ << "," << request.ball_position_.z_ << ",,";
            stream << request.goal_position_.x_ << "," << request.goal_position_.y_ << "," << request.goal_position_.z_ << ",,";
            stream << request.head_position_.x_ << "," << request.head_position_.y_ << "," << request.head_position_.z_ << ",,";
            stream << request.left_hand_position_.x_ << "," << request.left_hand_position_.y_ << "," << request.left_hand_position_.z_ << ",";
            stream << request.left_hand_position_.roll_ << "," << request.left_hand_position_.pitch_ << "," << request.left_hand_position_.yaw_ << ",,";
            stream << request.right_hand_position_.x_ << "," << request.right_hand_position_.y_ << "," << request.right_hand_position_.z_ << ",";
            stream << request.right_hand_position_.roll_ << "," << request.right_hand_position_.pitch_ << "," << request.right_hand_position_.yaw_ << ",,";
            stream << request.left_feet_position_.x_ << "," << request.left_feet_position_.y_ << "," << request.left_feet_position_.z_ << ",";
            stream << request.left_feet_position_.roll_ << "," << request.left_feet_position_.pitch_ << "," << request.left_feet_position_.yaw_ << ",,";
            stream << request.right_feet_position_.x_ << "," << request.right_feet_position_.y_ << "," << request.right_feet_position_.z_ << ",";
            stream << request.right_feet_position_.roll_ << "," << request.right_feet_position_.pitch_ << "," << request.right_feet_position_.yaw_ << std::endl;
	        stream << engine->get_throw_points_as_string();

            double counter = 0.0;
            stream << "Time,";
            stream << "LH,x,y,z,roll,pitch,yaw,";
            stream << "RH,x,y,z,roll,pitch,yaw,";
            stream << "LF,x,y,z,roll,pitch,yaw,";
            stream << "RF,x,y,z,roll,pitch,yaw" << std::endl;

            for(auto & it : responses){
                stream << counter << ",";
                stream << build_data_from_transform(it.support_foot_to_left_hand_);
                stream << build_data_from_transform(it.support_foot_to_right_hand_);
                stream << build_data_from_transform(it.support_foot_to_left_foot_);
                stream << build_data_from_transform(it.support_foot_to_right_foot_);
                stream << std::endl;

                counter += 1/sp_node_parameter_->engine_frequency_;
	        }

	        sp_debug_publisher_->print_throw_points(stream.str());
	    }
    }

    std::string RosPublisherFacade::build_data_from_transform(tf2::Transform & transform){
        std::stringstream stream;

        stream << "," << transform.getOrigin().x();
        stream << "," << transform.getOrigin().y();
        stream << "," << transform.getOrigin().z();
        stream << "," << transform.getRotation().x();
        stream << "," << transform.getRotation().y();
        stream << "," << transform.getRotation().z() << ",";

        return stream.str();
    }

    void RosPublisherFacade::visualize_engine(ThrowEngine * engine){
        engine->visualize_curves(sp_visualizer_);
    }

	void RosPublisherFacade::publish_debug(ThrowResponse const & response, int8_t const & percentage_done, int8_t const & movement_stage){
		if(sp_node_parameter_->debug_active_){
			sp_debug_publisher_->publish_ik_debug(response, percentage_done, movement_stage);
		}
	}
} //bitbots_throw