#include "kicks/Kick.hpp"

Kick::Kick(std::shared_ptr<KickEngineParameter> sp_parameter)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_parameter = sp_parameter;
	m_sp_spline_container = std::make_shared<bitbots_splines::SplineContainer>(init_trajectories());
}

std::shared_ptr<bitbots_splines::SplineContainer> Kick::create_trajectories(KickAttributes & kick_attributes)
{
	//TODO: testing
	//TODO: cleanup

	//checks
	if (check_requirements(kick_attributes))
	{
		// implement ROS Error
		return std::shared_ptr<bitbots_splines::SplineContainer>();
	}

	// build and return spline container
	double d_time = 0.0;
	bool b_prepared_kick = false;

	if (kick_attributes.prepare_kick_movement)
	{
		b_prepared_kick = calculate_movement_kick_preparation(d_time, kick_attributes.foot_starting_position, kick_attributes.foot_prepare_for_kick_position, kick_attributes.kick_ball_with_right);
	}

	calculate_movement_kick(d_time, b_prepared_kick ? kick_attributes.foot_prepare_for_kick_position : kick_attributes.foot_starting_position, kick_attributes.ball_position, kick_attributes.kick_goal_position, kick_attributes.kick_ball_with_right);

	if(kick_attributes.conclude_kick_movement)
	{
		calculate_movement_kick_conclusion(d_time, kick_attributes.ball_position, kick_attributes.foot_ending_position, kick_attributes.kick_ball_with_right);
	}

	return m_sp_spline_container;
}

bool Kick::use_default_calculation_for_kick_stating_position()
{
	//TODO: testing

	/*
	 * option for no or own calculation of the stating position for a kick.
	 * if overriden and returns false, the standard calculation will not be performed.
	 */

	return true;
}

bool Kick::check_foot_position_with_double_support(struct3d &  foot_starting_position)
{
	//TODO: testing
	//TODO: cleanup

	return foot_starting_position.x == 0.0 && foot_starting_position.y == 0.0 && foot_starting_position.z == 0.0;
}

void Kick::point(bitbots_splines::CurvePurpose spline_purpose, double time, double position, double velocity, double acceleration)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_spline_container->get(spline_purpose)->addPoint(time, position, velocity, acceleration);
}
