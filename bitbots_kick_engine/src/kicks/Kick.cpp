#include "Kick.hpp"

Kick::Kick(std::shared_ptr<KickEngineParameter> sp_parameter)
{
	m_sp_parameter = sp_parameter;
	m_sp_spline_container = init_trajectories();
}

std::shared_ptr<bitbots_splines::SplineContainer> Kick::create_trajectories(KickParameter& kick_parameter)
{
	//TODO: testing
	//TODO: cleanup

	//checks
	if (check_requirements(kick_parameter))
	{
		// implement ROS Error
		return std::shared_ptr<bitbots_splines::SplineContainer>();
	}

	// build and return spline container
	double d_time = 0.0;
	bool b_prepared_kick = false;

	if (kick_parameter.has_kick_prepareration())
	{
		b_prepared_kick = calculate_movement_kick_preparation(d_time, kick_parameter.get_foot_starting_position(), kick_parameter.get_foot_position_for_kick(), kick_parameter.kick_with_right());
	}

	calculate_movement_kick(d_time, b_prepared_kick ? kick_parameter.get_foot_position_for_kick() : kick_parameter.get_foot_starting_position(), kick_parameter.get_ball_position(), kick_parameter.get_kick_goal_position(), kick_parameter.kick_with_right());

	if(kick_parameter.has_kick_conclusion())
	{
		calculate_movement_kick_conclusion(d_time, kick_parameter.get_ball_position(), kick_parameter.get_foot_ending_position(), kick_parameter.kick_with_right());
	}

	return m_sp_spline_container;
}

bool Kick::check_foot_position_with_double_support(struct3d const& const foot_starting_position)
{
	return foot_starting_position.x == 0.0 && foot_starting_position.y == 0.0 && foot_starting_position.z == 0.0;
}

void Kick::point(bitbots_splines::CurvePurpose spline_purpose, double time, double position, double velocity, double acceleration)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_spline_container->get(spline_purpose)->addPoint(time, position, velocity, acceleration);
}
