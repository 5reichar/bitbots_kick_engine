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

bitbots_splines::SplineContainer * Kick::create_trajectories()
{
	//TODO: testing
	//TODO: cleanup

	auto trajectories = calculate_trajectories();
	bitbots_splines::SplineContainer* container = &trajectories;

	if (!m_sp_kick_start_position)
	{
		// implement ROS Error
		delete container;
		container = nullptr;
	}
	if (!m_sp_ball_position)
	{
		// implement ROS Error
		delete container;
		container = nullptr;
	}
	if (!m_sp_goal_position)
	{
		// implement ROS Error
		delete container;
		container = nullptr;
	}
	if (!additional_requirements())
	{
		delete container;
		container = nullptr;
	}

	return container;
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
	return true;
}
