#include "ros_interface/publisher/ros_publisher_facade.h"

RosPublisherFacade::RosPublisherFacade(ros::NodeHandle & ros_node_handle)
{
	// TODO testing
	// TODO cleanup

	sp_controller_command_publisher_.reset(new ControllerCommandPublisher(ros_node_handle));
	sp_odometry_publisher_.reset(new OdometryPublisher(ros_node_handle));
	sp_support_publisher_.reset(new SupportPublisher(ros_node_handle));
	sp_debug_publisher_.reset(new DebugPublisher(ros_node_handle));
}

void RosPublisherFacade::publish_throw(bool debug_active)
{
    sp_controller_command_publisher_->publish();
	sp_support_publisher_->publish();

	if(debug_active)
	{
		sp_debug_publisher_->publish();
		sp_debug_publisher_->publish_markers();
	}
}

void RosPublisherFacade::publish_odometry()
{
	sp_odometry_publisher_->publish();
}