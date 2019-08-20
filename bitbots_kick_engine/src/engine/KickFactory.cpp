#include "engine/KickFactory.hpp"
#include <math.h>

bitbots_splines::SplineContainer * KickFactory::make_kick_trajection(struct3d * ball_position, struct3d * goal_position)
{
	//TODO: testing
	//TODO: cleanup

	return make_kick_trajection(nullptr, ball_position, goal_position, nullptr);
}

bitbots_splines::SplineContainer * KickFactory::make_kick_trajection(struct3d * current_foot_position, struct3d * ball_position, struct3d * goal_position)
{
	//TODO: testing
	//TODO: cleanup

	return make_kick_trajection(current_foot_position, ball_position, goal_position, nullptr);
}

bitbots_splines::SplineContainer * KickFactory::make_kick_trajection(struct3d * current_foot_position, struct3d * ball_position, struct3d * goal_position, struct3d * final_foot_position)
{
	//TODO: testing
	//TODO: cleanup

	init(current_foot_position, ball_position, goal_position, final_foot_position);

	std::unique_ptr<Kick> up_kick(create_kick());

	up_kick->set_foot_position(m_sp_current_foot_position);
	up_kick->set_kick_start_position(m_sp_kick_start_foot_position);
	up_kick->set_ball_position(m_sp_current_ball_position);
	up_kick->set_foot_end_position(m_sp_final_foot_position);
	up_kick->set_kick_with_right(m_b_kick_with_right);

	reset();

	return up_kick->create_trajectories();
}

void KickFactory::init(struct3d* current_foot_position, struct3d* ball_position, struct3d* goal_position, struct3d* final_foot_position)
{
	//TODO: testing
	//TODO: cleanup

	m_d_angle_robot_ball = calculate_angle(ball_position->x, ball_position->y);
	m_d_angle_ball_goal = calculate_angle(goal_position->x - ball_position->x, goal_position->y - ball_position->y);

	m_sp_current_foot_position.reset(current_foot_position);
	m_sp_kick_start_foot_position.reset(calculate_kick_start());
	m_sp_current_ball_position.reset(ball_position);
	m_sp_ball_goal_position.reset(goal_position);
	m_sp_final_foot_position.reset(final_foot_position);

	m_b_kick_with_right = check_kicking_with_right();
}

void KickFactory::reset()
{
	//TODO: testing
	//TODO: cleanup

	m_sp_current_foot_position.reset();
	m_sp_kick_start_foot_position.reset();
	m_sp_current_ball_position.reset();
	m_sp_ball_goal_position.reset();
}

double KickFactory::calculate_angle(double const x, double const y)
{
	//TODO: testing
	//TODO: cleanup

	double angle = y == 0.0 ? 0.0 : std::atan(std::abs(x) / std::abs(y));

	if (x <= 0.0 && y > 0.0)
	{
		angle += 270.0;
	}
	else if (x < 0.0 && y <= 0.0)
	{
		angle += 180.0;
	}
	else if (x >= 0.0 && y < 0.0)
	{
		angle += 90.0;
	}

	return angle;
}

struct3d * KickFactory::calculate_kick_start()
{
	//TODO: make dynamic (adjustable via KickEngineParameter and/or Kick)
	//TODO: testing
	//TODO: cleanup

	struct3d* kick_start_position = new struct3d();

	if (m_d_angle_robot_ball > 315 || m_d_angle_robot_ball < 45)
	{
		// TODO: Replace placeholder-values
		kick_start_position->x = 0;
		kick_start_position->y = -10;
		kick_start_position->z = -10;
	}
	else if (m_d_angle_robot_ball > 135 || m_d_angle_robot_ball < 225)
	{
		// TODO: Replace placeholder-values
		kick_start_position->x = 0;
		kick_start_position->y = 10;
		kick_start_position->z = 5;
	}
	else if (m_d_angle_robot_ball == 90 || m_d_angle_robot_ball == 270)
	{
		// TODO: Replace placeholder-values
		kick_start_position->x = 0;
		kick_start_position->y = 0;
		kick_start_position->z = 0;
	}
	else if (m_d_angle_robot_ball > 45 || m_d_angle_robot_ball < 90)
	{
		// TODO: Replace placeholder-values
		kick_start_position->x = -5;
		kick_start_position->y = 5;
		kick_start_position->z = 0;
	}
	else if (m_d_angle_robot_ball > 90 || m_d_angle_robot_ball < 135)
	{
		// TODO: Replace placeholder-values
		kick_start_position->x = -5;
		kick_start_position->y = -5;
		kick_start_position->z = 0;
	}
	else if (m_d_angle_robot_ball > 225 || m_d_angle_robot_ball < 270)
	{
		// TODO: Replace placeholder-values
		kick_start_position->x = 5;
		kick_start_position->y = -5;
		kick_start_position->z = 0;
	}
	else if (m_d_angle_robot_ball > 270 || m_d_angle_robot_ball < 315)
	{
		// TODO: Replace placeholder-values
		kick_start_position->x = 5;
		kick_start_position->y = 5;
		kick_start_position->z = 0;
	}

	return kick_start_position;
}

bool KickFactory::check_kicking_with_right()
{
	//TODO: make dynamic (adjustable via KickEngineParameter and/or Kick)
	//TODO: testing
	//TODO: cleanup

	return m_d_angle_robot_ball < 180;
}

Kick * KickFactory::create_kick()
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	Kick * p_kick = nullptr;



	return p_kick;
}
