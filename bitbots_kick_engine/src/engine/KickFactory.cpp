#include <KickFactory.hpp>

#include <math.h>
#include "../kicks/SmoothSplineKick.hpp"

KickFactory::KickFactory(std::shared_ptr<KickEngineParameter> sp_parameter)
{
	//TODO: testing

	m_sp_kick_engine_parameter = sp_parameter;
}

KickParameter KickFactory::get_last_kicks_parameter()
{
	return m_struc_kick_parameter;
}

bitbots_splines::SplineContainer * KickFactory::make_kick_trajection(struct3d * ball_position, struct3d * goal_position, struct3d * final_foot_position)
{
	//TODO: testing
	//TODO: cleanup

	init_kick_parameter(ball_position, goal_position, final_foot_position);

	auto sp_kick = create_kick();

	if (sp_return_kick->use_default_calculation_for_kick_stating_position())
	{
		m_struc_kick_parameter.foot_position_for_starting_kick = KickFactoryService::calculate_kick_start(m_struc_kick_parameter.angle_between_robot_and_ball, m_struc_kick_parameter.angle_between_ball_and_goal);
		m_struc_kick_parameter.prepare_kick_movement = true;
	}

	bool can_make_trajectories = sp_kick && check_generale_requirements();

	return can_make_trajectories ? sp_kick->create_trajectories(m_struc_kick_parameter) : nullptr;
}

KickParameter KickFactory::init_kick_parameter(struct3d* ball_position, struct3d* goal_position, struct3d* final_foot_position)
{
	//TODO: testing
	//TODO: cleanup

	m_struc_kick_parameter.angle_between_robot_and_ball = KickFactoryService::calculate_angle(ball_position->x, ball_position->y);
	m_struc_kick_parameter.angle_between_ball_and_goal = KickFactoryService::calculate_angle(goal_position->x - ball_position->x, goal_position->y - ball_position->y);

	m_struc_kick_parameter.kick_ball_with_right = KickFactoryService::check_kicking_with_right(angle_between_robot_and_ball);

	m_struc_kick_parameter.foot_starting_position = m_struc_kick_parameter.kick_ball_with_right ? get_current_right_foot_position() : get_current_left_foot_position();
	m_struc_kick_parameter.ball_position = ball_position;
	m_struc_kick_parameter.kick_goal_position = goal_position;

	if (final_foot_position == nullptr)
	{
		m_struc_kick_parameter.conclude_kick_movement = false;
	}
	else
	{
		m_struc_kick_parameter.foot_ending_position = final_foot_position;
		m_struc_kick_parameter.conclude_kick_movement = true;
	}
}

bool KickFactory::check_generale_requirements()
{
	//TODO: testing
	//TODO: cleanup

	bool requirements_meet = true;

	// is Robot in the way
	requirements_meet &= !(m_struc_kick_parameter.get_angle_between_robot_and_ball() == 90 && m_struc_kick_parameter.get_angle_between_ball_and_goal() == 270);
	requirements_meet &= !(m_struc_kick_parameter.get_angle_between_robot_and_ball() == 270 && m_struc_kick_parameter.get_angle_between_ball_and_goal() == 90);

	return requirements_meet;
}

struct3d KickFactory::get_current_left_foot_position() const
{
	//TODO: implementation
	//TODO: testing
	//TODO: cleanup

	return struct3d();
}

struct3d KickFactory::get_current_right_foot_position() const
{
	//TODO: implementation
	//TODO: testing
	//TODO: cleanup

	return struct3d();
}

std::shared_ptr<Kick> KickFactory::create_kick()
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	std::shared_ptr<Kick> sp_return_kick;

	if (true)
	{
		//TODO: impolement test for when to use this Kick
		sp_return_kick = std::make_shared< SmoothSplineKick>(m_sp_kick_engine_parameter);
	}

	return sp_return_kick;
}
