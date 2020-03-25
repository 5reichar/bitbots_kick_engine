#include "ros_interface/throw_node.h"

ThrowNode::ThrowNode()
{
	// TODO testing
	// TODO cleanup

	bool simulation = false;
	ros_node_handle_.param<bool>("/simulation_active", simulation, false);

	up_publisher_facade_.reset(new RosPublisherFacade(ros_node_handle_));
	init_ros_subcribtions();
}

void ThrowNode::init_ros_subcribtions()
{
	// TODO testing
	// TODO cleanup

	ros_subsciber_throw_ = ros_node_handle_.subscribe("throw", 1, &ThrowNode::throw_callback, this, ros::TransportHints().tcpNoDelay());
}

void ThrowNode::throw_callback(const bitbots_throw_engine::throw_action action)
{
	// TODO testing
	// TODO cleanup

	struct3d ball_position = {action.ball_position.x, action.ball_position.y, action.ball_position.z };
	struct3d goal_position = {action.throw_target.x, action.throw_target.y, action.throw_target.z };

	uint16_t odometry_counter = 1;

	ros::Rate loopRate(sp_node_parameter_->engine_frequency_);

	while (ros::ok())
	{
		if (sp_throw_engine_->throw_ball(ball_position, goal_position))
		{
			up_publisher_facade_->publish_throw(sp_node_parameter_->debug_active_);
		}

		if (odometry_counter > sp_node_parameter_->odom_publish_factor_)
		{
			up_publisher_facade_->publish_odometry();
			odometry_counter = 1;
		}
		else
		{
			++odometry_counter;
		}

		ros::spinOnce();
		loopRate.sleep();
	}

}