#include "kicks/Kick.hpp"

Kick::Kick(std::shared_ptr<KickEngineParameter> sp_parameter)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_parameter = sp_parameter;
}

std::shared_ptr<bitbots_splines::SplineContainer> Kick::calculateTrajectories(KickAttributes & kick_attributes)
{
	//TODO: testing
	//TODO: cleanup

	//checks
	if (checkRequirements(kick_attributes))
	{
		// implement ROS Error
		return std::shared_ptr<bitbots_splines::SplineContainer>();
	}

	// build and return spline container
	double d_time = 0.0;
	bool b_prepared_kick = false;

	if (kick_attributes.prepare_kick_movement)
	{
		b_prepared_kick = calculateMovementKickPreparation(d_time, kick_attributes.foot_starting_position, kick_attributes.foot_prepare_for_kick_position, kick_attributes.kick_ball_with_right);
	}

	calculateMovementKick(d_time, b_prepared_kick ? kick_attributes.foot_prepare_for_kick_position : kick_attributes.foot_starting_position, kick_attributes.ball_position, kick_attributes.kick_goal_position, kick_attributes.kick_ball_with_right);

	if(kick_attributes.conclude_kick_movement)
	{
		calculateMovementKickConclusion(d_time, kick_attributes.ball_position, kick_attributes.foot_ending_position, kick_attributes.kick_ball_with_right);
	}

	return m_sp_spline_container;
}

bool Kick::useDefaultCalculationForKickStatingPosition()
{
	//TODO: testing

	/*
	 * option for no or own calculation of the stating position for a kick.
	 * if overriden and returns false, the standard calculation will not be performed.
	 */

	return true;
}

bool Kick::checkFootPositionWithDoubleSupport(struct3d &  foot_starting_position)
{
	//TODO: testing
	//TODO: cleanup

	return foot_starting_position.x == 0.0 && foot_starting_position.y == 0.0 && foot_starting_position.z == 0.0;
}

void Kick::point(bitbots_splines::CurvePurpose spline_purpose, double time, double position, double velocity, double acceleration)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_spline_container->get(spline_purpose)->add_point(time, position, velocity, acceleration);
}
