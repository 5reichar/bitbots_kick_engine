#include "ros_interface/publisher/ros_publisher_facade.h"

RosPublisherFacade::RosPublisherFacade(ros::NodeHandle & ros_node_handle, std::shared_ptr<ThrowNodeParameter> parameter)
	: sp_node_parameter_(parameter)
{
	// TODO testing
	// TODO cleanup

	sp_controller_command_publisher_.reset(new ControllerCommandPublisher(ros_node_handle));
	sp_odometry_publisher_.reset(new OdometryPublisher(ros_node_handle));
	sp_support_publisher_.reset(new SupportPublisher(ros_node_handle));
	sp_debug_publisher_.reset(new DebugPublisher(ros_node_handle));
}

void RosPublisherFacade::publish_throw()
{
	auto time = ros::Time::now();

	// TODO Replace Placeholder
    sp_controller_command_publisher_->publish(time, std::vector<std::string>(), std::vector<double>());
	sp_support_publisher_->publish();

	if(sp_node_parameter_->debug_active_)
	{
		sp_debug_publisher_->publish();
		sp_debug_publisher_->publish_markers();
	}
}

void RosPublisherFacade::publish_odometry()
{
	sp_odometry_publisher_->publish(sp_node_parameter_->odom_publish_factor_);
}

void RosPublisherFacade::prepare_publisher_for_throw()
{
	sp_odometry_publisher_->reset_counter();
}