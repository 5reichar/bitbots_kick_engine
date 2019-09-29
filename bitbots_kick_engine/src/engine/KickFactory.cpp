#include <KickFactory.hpp>

#include <math.h>
#include <SmoothSplineKick.hpp>

KickFactory::KickFactory(std::shared_ptr<KickEngineParameter> sp_parameter)
{
	m_sp_parameter = sp_parameter;

	north = 0.0;
	north_east = 45.0;
	east = 90.0;
	south_east = 135.0;
	south = 180.0;
	south_west = 225.0;
	west = 270.0;
	north_west = 315.0;
}


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

	auto parameter = init(current_foot_position, ball_position, goal_position, final_foot_position);
	auto sp_kick = create_kick();
	bool can_make_trajectories = sp_kick && check_generale_requirements();

	return can_make_trajectories ? sp_kick->create_trajectories(parameter) : nullptr;
}

KickParameter KickFactory::init(struct3d* current_foot_position, struct3d* ball_position, struct3d* goal_position, struct3d* final_foot_position)
{
	//TODO: testing
	//TODO: cleanup

	auto angle_between_robot_and_ball = calculate_angle(ball_position->x, ball_position->y);
	auto angle_between_ball_and_goal = calculate_angle(goal_position->x - ball_position->x, goal_position->y - ball_position->y);

	KickParameter parameter;
	parameter.set_foot_starting_position(current_foot_position);
	parameter.set_foot_position_for_kick(calculate_kick_start(angle_between_robot_and_ball, angle_between_ball_and_goal));
	parameter.set_ball_position(ball_position);
	parameter.set_kick_goal_position(goal_position);
	parameter.set_foot_ending_position(final_foot_position);

	parameter.set_angle_between_robot_and_ball(get_angle_between_robot_and_ball);
	parameter.set_angle_between_ball_and_goal(angle_between_ball_and_goal);

	parameter.set_kick_with_right(check_kicking_with_right(angle_between_robot_and_ball));

	return parameter;
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

struct3d* KickFactory::calculate_kick_start(double angle_between_robot_and_ball, double angle_between_ball_and_goal)
{
	/*
			  y (front)             z (up)
				  /\				  /\
				   |				   |
				   |				   |
				   |				   |
	-x (left)<-----+-----> x (right)   0
				   |				   |
				   |				   |
				   |				   |
				  \/		          \/
			  -y (back)            -z (down)
	*/

	//TODO: make dynamic (adjustable via KickEngineParameter and/or Kick)
	//TODO: testing
	//TODO: cleanup

	struct3d* kick_start_position = new struct3d();

	if (angle_between_robot_and_ball > north_west || angle_between_robot_and_ball <= north_east)
	{
		// north sector
		if (angle_between_ball_and_goal > north_west || angle_between_ball_and_goal <= north_east)
		{
			// north direction
			set_straight_kick_position_front(kick_start_position);
		}
		else if (angle_between_ball_and_goal > north_east || angle_between_ball_and_goal <= south_east)
		{
			// east direction
			set_side_kick_position_front_left(kick_start_position);
		}
		else if (angle_between_ball_and_goal > south_east || angle_between_ball_and_goal <= south_west)
		{
			// south direction
			set_straight_kick_position_back(kick_start_position);
		}
		else if (angle_between_ball_and_goal > south_west || angle_between_ball_and_goal <= north_west)
		{
			// west direction
			set_side_kick_position_front_right(kick_start_position);
		}
	}
	else if (angle_between_robot_and_ball > north_east || angle_between_robot_and_ball <= south_east)
	{
		// east sector
		if (angle_between_ball_and_goal > north_west || angle_between_ball_and_goal <= north_east)
		{
			// north direction
			set_side_kick_position_back_right(kick_start_position);
		}
		else if (angle_between_ball_and_goal > north_east || angle_between_ball_and_goal <= south_east)
		{
			// east direction
			set_default_kick_position(kick_start_position);
		}
		else if (angle_between_ball_and_goal > south_east || angle_between_ball_and_goal <= south_west)
		{
			// south direction
			set_side_kick_position_front_right(kick_start_position);
		}
		else if (angle_between_ball_and_goal > south_west || angle_between_ball_and_goal <= north_west)
		{
			// west direction
			set_straight_kick_position_right(kick_start_position);
		}
	}
	else if (angle_between_robot_and_ball > south_east || angle_between_robot_and_ball <= south_west)
	{
		// south sector
		if (angle_between_ball_and_goal > north_west || angle_between_ball_and_goal <= north_east)
		{
			// north direction
			set_straight_kick_position_back(kick_start_position);
		}
		else if (angle_between_ball_and_goal > north_east || angle_between_ball_and_goal <= south_east)
		{
			// east direction
			set_side_kick_position_back_left(kick_start_position);
		}
		else if (angle_between_ball_and_goal > south_east || angle_between_ball_and_goal <= south_west)
		{
			// south direction
			set_straight_kick_position_front(kick_start_position);
		}
		else if (angle_between_ball_and_goal > south_west || angle_between_ball_and_goal <= north_west)
		{
			// west direction
			set_side_kick_position_back_right(kick_start_position);
		}
	}
	else if (angle_between_robot_and_ball > south_west || angle_between_robot_and_ball <= north_west)
	{
		// west sector
		if (angle_between_ball_and_goal > north_west || angle_between_ball_and_goal <= north_east)
		{
			// north direction
			set_side_kick_position_back_left(kick_start_position);
		}
		else if (angle_between_ball_and_goal > north_east || angle_between_ball_and_goal <= south_east)
		{
			// east direction
			set_straight_kick_position_left(kick_start_position);
		}
		else if (angle_between_ball_and_goal > south_east || angle_between_ball_and_goal <= south_west)
		{
			// south direction
			set_side_kick_position_front_left(kick_start_position);
		}
		else if (angle_between_ball_and_goal > south_west || angle_between_ball_and_goal <= north_west)
		{
			// west direction
			set_default_kick_position(kick_start_position);
		}
	}

	return kick_start_position;
}

void KickFactory::set_default_kick_position(struct3d* kick_position)
{
	// TODO: Replace placeholder-values
	kick_position->x = 0;
	kick_position->y = 0;
	kick_position->z = 0;
}

void KickFactory::set_straight_kick_position_front(struct3d* kick_position)
{
	// TODO: Replace placeholder-values
	kick_position->x = 0;
	kick_position->y = 10;
	kick_position->z = 5;
}

void KickFactory::set_straight_kick_position_back(struct3d* kick_position)
{
	// TODO: Replace placeholder-values
	kick_position->x = 0;
	kick_position->y = -10;
	kick_position->z = 10;
}

void KickFactory::set_straight_kick_position_left(struct3d* kick_position)
{
	// TODO: Replace placeholder-values
	kick_position->x = -10;
	kick_position->y = 0;
	kick_position->z = 10;
}

void KickFactory::set_straight_kick_position_right(struct3d* kick_position)
{
	// TODO: Replace placeholder-values
	kick_position->x = 10;
	kick_position->y = 0;
	kick_position->z = 10;
}

void KickFactory::set_side_kick_position_front_left(struct3d* kick_position)
{
	// TODO: Replace placeholder-values
	kick_position->x = -5;
	kick_position->y = 5;
	kick_position->z = 0;
}

void KickFactory::set_side_kick_position_back_left(struct3d* kick_position)
{
	// TODO: Replace placeholder-values
	kick_position->x = -5;
	kick_position->y = -5;
	kick_position->z = 0;
}

void KickFactory::set_side_kick_position_front_right(struct3d* kick_position)
{
	// TODO: Replace placeholder-values
	kick_position->x = 5;
	kick_position->y = -5;
	kick_position->z = 0;
}

void KickFactory::set_side_kick_position_back_right(struct3d* kick_position)
{
	// TODO: Replace placeholder-values
	kick_position->x = 5;
	kick_position->y = 5;
	kick_position->z = 0;
}

bool KickFactory::check_kicking_with_right(double angle_between_robot_and_ball)
{
	//TODO: make dynamic (adjustable via KickEngineParameter and/or Kick)
	//TODO: testing
	//TODO: cleanup

	return angle_between_robot_and_ball < 180;
}

bool KickFactory::check_generale_requirements(KickParameter& kick_parameter)
{
	bool requirements_meet = true;

	// is Robot in the way
	requirements_meet &= !(kick_parameter.get_angle_between_robot_and_ball() == this->east && kick_parameter.get_angle_between_ball_and_goal() == this->west);
	requirements_meet &= !(kick_parameter.get_angle_between_robot_and_ball() == this->west && kick_parameter.get_angle_between_ball_and_goal() == this->east);

	return requirements_meet;
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
		sp_return_kick.reset(new SmoothSplineKick(m_sp_parameter));
	}

	return sp_return_kick;
}
