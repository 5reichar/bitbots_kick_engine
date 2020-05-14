#include "ros_interface/publisher/ros_publisher_facade.h"

RosPublisherFacade::RosPublisherFacade(ros::NodeHandle & ros_node_handle, std::shared_ptr<ThrowNodeParameter> parameter)
	: sp_node_parameter_(parameter)
{
	//TODO: testing
	//TODO: cleanup

	sp_controller_command_publisher_.reset(new ControllerCommandPublisher(ros_node_handle, "throwing_motor_goals"));
	sp_odometry_publisher_.reset(new OdometryPublisher(ros_node_handle));
	sp_support_publisher_.reset(new SupportPublisher(ros_node_handle));
	sp_debug_publisher_.reset(new DebugPublisher(ros_node_handle));
}

void RosPublisherFacade::prepare_publisher_for_throw()
{
	sp_odometry_publisher_->reset_counter();
}

void RosPublisherFacade::publish_throw(bitbots_splines::JointGoals & joint_goals)
{
	auto time = ros::Time::now();

    sp_controller_command_publisher_->publish(time, joint_goals);
	sp_support_publisher_->publish();
}

void RosPublisherFacade::publish_odometry()
{
	sp_odometry_publisher_->publish(sp_node_parameter_->odom_publish_factor_);
}

void RosPublisherFacade::publish_debug()
{
	if(sp_node_parameter_->debug_active_)
	{
		sp_debug_publisher_->publish();
		sp_debug_publisher_->publish_markers();
	}
}