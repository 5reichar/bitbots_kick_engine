#include "KickFactory.hpp"

bitbots_splines::SplineContainer KickFactory::make_kick_trajection(struct3d & const ball, struct3d & const goal)
{
	//TODO: testing
	//TODO: cleanup

	init(ball, goal);

	auto kick_trajectory = make_kick_trajection();

	reset();

	return kick_trajectory;
}

void KickFactory::init(struct3d& const ball, struct3d& const goal)
{
	//TODO: testing
	//TODO: cleanup

	m_d_angle_robot_ball = calculate_angle(ball.x, ball.y);
	m_d_angle_ball_goal = calculate_angle(goal.x - ball.x, goal.y - ball.y);

	m_pc_current_foot_position = get_foot_position();
	m_pc_kick_start_foot_position = calculate_kick_start();
	m_pc_current_ball_position = ball;
	m_pc_ball_goal_position = goal;
	m_b_kick_with_left = check_kicking_with_left();
}

void KickFactory::reset()
{
	//TODO: testing
	//TODO: cleanup

	delete m_pc_current_foot_position;
	delete m_pc_kick_start_foot_position;
	delete m_pc_current_ball_position;
	delete m_pc_ball_goal_position;
}

double KickFactory::calculate_angle(double const x, double const y)
{
	//TODO: testing
	//TODO: cleanup

	double angle = y == 0.0 ? std::atan(std::abs(x) / std::abs(y));

	if (x < 0 && y > 0)
	{
		angle += 270.0;
	}
	else if (x < 0 && y < 0)
	{
		angle += 180.0;
	}
	else if (x > 0 && y < 0)
	{
		angle += 90.0;
	}

	return angle;
}

struct3d KickFactory::calculate_kick_start()
{
	//TODO: make dynamic (adjustable via KickEngineParameter and/or Kick)
	//TODO: testing
	//TODO: cleanup

	struct3d kick_start_position;

	if (m_d_angle_robot_ball > 315 || m_d_angle_robot_ball < 45)
	{
		// TODO: Replace placeholder-values
		kick_start_position.x = 0;
		kick_start_position.y = -10;
		kick_start_position.z = -10;
	}
	else if (m_d_angle_robot_ball > 135 || m_d_angle_robot_ball < 225)
	{
		// TODO: Replace placeholder-values
		kick_start_position.x = 0;
		kick_start_position.y = 10;
		kick_start_position.z = 5;
	}
	else if (m_d_angle_robot_ball == 90 || m_d_angle_robot_ball == 270)
	{
		// TODO: Replace placeholder-values
		kick_start_position.x = 0;
		kick_start_position.y = 0;
		kick_start_position.z = 0;
	}
	else if (m_d_angle_robot_ball > 45 || m_d_angle_robot_ball < 90)
	{
		// TODO: Replace placeholder-values
		kick_start_position.x = -5;
		kick_start_position.y = 5;
		kick_start_position.z = 0;
	}
	else if (m_d_angle_robot_ball > 90 || m_d_angle_robot_ball < 135)
	{
		// TODO: Replace placeholder-values
		kick_start_position.x = -5;
		kick_start_position.y = -5;
		kick_start_position.z = 0;
	}
	else if (m_d_angle_robot_ball > 225 || m_d_angle_robot_ball < 270)
	{
		// TODO: Replace placeholder-values
		kick_start_position.x = 5;
		kick_start_position.y = -5;
		kick_start_position.z = 0;
	}
	else if (m_d_angle_robot_ball > 270 || m_d_angle_robot_ball < 315)
	{
		// TODO: Replace placeholder-values
		kick_start_position.x = 5;
		kick_start_position.y = 5;
		kick_start_position.z = 0;
	}

	return kick_start_position;
}

struct3d KickFactory::get_foot_position()
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	return struct3d();
}

bool KickFactory::check_kicking_with_left()
{
	//TODO: make dynamic (adjustable via KickEngineParameter and/or Kick)
	//TODO: testing
	//TODO: cleanup

	return m_d_angle_robot_ball > 180;
}

bitbots_splines::SplineContainer KickFactory::make_kick_trajection()
{
	//TODO: Implementation
	//TODO: testing
	//TODO: cleanup

	bitbots_splines::SplineContainer kick_trajection;



	return kick_trajection;
}
