#include "kicks/Kick.hpp"

void Kick::reset()
{
	//TODO: testing
	//TODO: cleanup

	m_sp_ball_position.reset();
	m_sp_foot_end_position.reset();
	m_sp_foot_position.reset();
	m_sp_goal_position.reset();
	m_sp_kick_start_position.reset();
}

std::shared_ptr<bitbots_splines::SplineContainer> Kick::create_trajectories()
{
	//TODO: testing
	//TODO: cleanup

	//checks
	bool checks_succ = true;

	if (!m_sp_kick_start_position)
	{
		// implement ROS Error
		checks_succ = false;
	}
	if (!m_sp_ball_position)
	{
		// implement ROS Error
		checks_succ = false;
	}
	if (!m_sp_goal_position)
	{
		// implement ROS Error
		checks_succ = false;
	}
	if (!additional_requirements())
	{
		checks_succ = false;
	}

	// build and return spline container
	if (checks_succ)
	{
		build_trajectories();
		return m_sp_spline_container;
	}
	else
	{
		return std::shared_ptr<bitbots_splines::SplineContainer>;
	}
}

void Kick::set_foot_position(std::shared_ptr<struct3d> position)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_foot_position = position;
}

void Kick::set_kick_start_position(std::shared_ptr<struct3d> position)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_kick_start_position = position;
}

void Kick::set_ball_position(std::shared_ptr<struct3d> position)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_ball_position = position;
}

void Kick::set_goal_position(std::shared_ptr<struct3d> position)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_goal_position = position;
}

void Kick::set_foot_end_position(std::shared_ptr<struct3d> position)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_foot_end_position = position;
}

void Kick::set_kick_with_right(bool kick_with_right)
{
	//TODO: testing
	//TODO: cleanup

	m_b_kick_with_right = kick_with_right;
}

bool Kick::additional_requirements()
{
	//TODO: testing
	//TODO: cleanup

	return true;
}

bitbots_splines::SplineContainer Kick::calculate_trajectories()
{
	return bitbots_splines::SplineContainer();
}

void Kick::point(bitbots_splines::CurvePurpose spline_purpose, double time, double position, double velocity, double acceleration)
{
	//TODO: testing
	//TODO: cleanup

	m_sp_spline_container->get(spline_purpose)->addPoint(time, position, velocity, acceleration);
}
