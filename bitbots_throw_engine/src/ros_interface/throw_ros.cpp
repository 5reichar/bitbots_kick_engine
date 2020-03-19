#include "ros_interface/throw_ros.h"

ThrowRos::ThrowRos(/* args */)
{

}

ThrowRos::~ThrowRos()
{

}

void ThrowRos::run_throw(struct3d & ball_position, struct3d & goal_position)
{
    // TODO testing
	// TODO cleanup

	uint16_t odometry_counter = 1;

	ros::Rate loopRate(sp_node_parameter_->engine_frequency_);

	while (ros::ok())
	{
		if (sp_throw_engine_->throw_ball(ball_position, goal_position))
		{
			//publishKick();
		}

		if (odometry_counter > sp_node_parameter_->odom_publish_factor_)
		{
			//publishOdemetry();
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