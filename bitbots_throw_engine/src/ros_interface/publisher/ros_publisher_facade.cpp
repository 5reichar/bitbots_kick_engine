#include "ros_interface/publisher/ros_publisher_facade.h"
#include "ros_interface/publisher/system_publisher.h"
#include "ros_interface/ros_joint_and_topic_names.h"

namespace bitbots_throw{
	RosPublisherFacade::RosPublisherFacade(ros::NodeHandle & ros_node_handle
                                          ,std::shared_ptr<ThrowEngineParameter> & sp_engine_parameter
	                                      ,ThrowVisualizer::ThrowVisualizerParams const & visualization_parameter
                                          ,std::shared_ptr<ThrowDebugParameter> & sp_debug_parameter)
		: sp_engine_parameter_(sp_engine_parameter)
		, sp_debug_parameter_(sp_debug_parameter)
		, sp_visualizer_(new ThrowVisualizer(RosJointAndTopicNames::get_topic_debug_visualization_base(), visualization_parameter, sp_debug_parameter))
		, sp_controller_command_publisher_(new ControllerCommandPublisher(ros_node_handle, RosJointAndTopicNames::get_topic_controller_command()))
		, sp_odometry_publisher_(new OdometryPublisher(ros_node_handle, RosJointAndTopicNames::get_topic_odometry()))
		, sp_support_publisher_(new SupportPublisher(ros_node_handle, RosJointAndTopicNames::get_topic_support()))
		, sp_debug_publisher_(new DebugPublisher(ros_node_handle, RosJointAndTopicNames::get_topic_debug(), RosJointAndTopicNames::get_topic_debug_marker())){
	}

	void RosPublisherFacade::prepare_publisher_for_throw(){
		sp_odometry_publisher_->reset_counter();
	}

    void RosPublisherFacade::set_parameter(std::shared_ptr<ThrowEngineParameter> & sp_engine_parameter, std::shared_ptr<ThrowDebugParameter> & sp_debug_parameter){
        sp_debug_parameter_ = sp_debug_parameter;
        sp_engine_parameter_ = sp_engine_parameter;
        sp_visualizer_->set_parameter(sp_debug_parameter);
    }

	void RosPublisherFacade::publish_throw(bitbots_splines::JointGoals & joint_goals){
		auto time = ros::Time::now();

		sp_controller_command_publisher_->publish(time, joint_goals);
		sp_support_publisher_->publish();
	}

	void RosPublisherFacade::publish_odometry(){
		sp_odometry_publisher_->publish(sp_engine_parameter_->odom_publish_factor_);
	}

    void RosPublisherFacade::publish_engine_debug(ThrowEngine * engine, ThrowRequest const & request, std::vector<ThrowResponse> & responses){
	    if(sp_debug_parameter_->debug_active_){
	        SystemPublisher::publish_info("Printing Engine data", "RosPublisherFacade");
	        std::stringstream stream;

            stream << "Throw Request" << std::endl;
            stream << "Position,x,y,z,roll,pitch,yaw" << std::endl;
            stream << "Ball" << request.ball_position_.x_ << "," << request.ball_position_.y_ << "," << request.ball_position_.z_ << std::endl;
            stream << "Goal" << request.goal_position_.x_ << "," << request.goal_position_.y_ << "," << request.goal_position_.z_ << std::endl;

            for(auto it : request.joint_start_position_){
                stream << get_robot_joint_string(it.first);
                stream << "," << it.second.x_;
                stream << "," << it.second.y_;
                stream << "," << it.second.z_;
                stream << "," << it.second.roll_;
                stream << "," << it.second.pitch_;
                stream << "," << it.second.yaw_;
                stream << std::endl;
            }
	        stream << engine->get_throw_points_as_string();

            double counter = 0.0;
            stream << "Time, IKMode,";
            stream << "LH,x,y,z,roll,pitch,yaw,";
            stream << "RH,x,y,z,roll,pitch,yaw,";
            stream << "LF,x,y,z,roll,pitch,yaw,";
            stream << "RF,x,y,z,roll,pitch,yaw" << std::endl;

            for(auto & it : responses){
                stream << counter << "," << it.ik_mode_ << ",";
                stream << build_data_from_transform(it.transform_to_joint_[RobotJoints::left_hand]);
                stream << build_data_from_transform(it.transform_to_joint_[RobotJoints::right_hand]);
                stream << build_data_from_transform(it.transform_to_joint_[RobotJoints::left_foot]);
                stream << build_data_from_transform(it.transform_to_joint_[RobotJoints::right_foot]);
                stream << std::endl;

                counter += 1 / sp_engine_parameter_->engine_frequency_;
	        }

	        sp_debug_publisher_->print_throw_points(stream.str(), sp_debug_parameter_);
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
		if(sp_debug_parameter_->debug_active_){
			sp_debug_publisher_->publish_ik_debug(response, percentage_done, movement_stage);
		}
	}
} //bitbots_throw