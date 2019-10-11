#include "..\..\include\engine\KickFactoryService.hpp"
#include "..\..\include\kicks\KickParameter.hpp"

const double KickFactoryService::north = 0.0;
const double KickFactoryService::north_east = 45.0;
const double KickFactoryService::east = 90.0;
const double KickFactoryService::south_east = 135.0;
const double KickFactoryService::south = 180.0;
const double KickFactoryService::south_west = 225.0;
const double KickFactoryService::west = 270.0;
const double KickFactoryService::north_west = 315.0;

double KickFactoryService::calculate_angle(double const x, double const y)
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

struct3d KickFactoryService::calculate_kick_start(double angle_between_robot_and_ball, double angle_between_ball_and_goal)
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

	struct3d kick_start_position;

	if (angle_between_robot_and_ball > north_west || angle_between_robot_and_ball <= north_east)
	{
		// north sector
		if (angle_between_ball_and_goal > north_west || angle_between_ball_and_goal <= north_east)
		{
			// north direction
			kick_start_position = get_straight_kick_position_front();
		}
		else if (angle_between_ball_and_goal > north_east || angle_between_ball_and_goal <= south_east)
		{
			// east direction
			kick_start_position = get_side_kick_position_front_left();
		}
		else if (angle_between_ball_and_goal > south_east || angle_between_ball_and_goal <= south_west)
		{
			// south direction
			kick_start_position = get_straight_kick_position_back();
		}
		else if (angle_between_ball_and_goal > south_west || angle_between_ball_and_goal <= north_west)
		{
			// west direction
			kick_start_position = get_side_kick_position_front_right();
		}
	}
	else if (angle_between_robot_and_ball > north_east || angle_between_robot_and_ball <= south_east)
	{
		// east sector
		if (angle_between_ball_and_goal > north_west || angle_between_ball_and_goal <= north_east)
		{
			// north direction
			kick_start_position = get_side_kick_position_back_right();
		}
		else if (angle_between_ball_and_goal > north_east || angle_between_ball_and_goal <= south_east)
		{
			// east direction
			kick_start_position = get_default_kick_position();
		}
		else if (angle_between_ball_and_goal > south_east || angle_between_ball_and_goal <= south_west)
		{
			// south direction
			kick_start_position = get_side_kick_position_front_right();
		}
		else if (angle_between_ball_and_goal > south_west || angle_between_ball_and_goal <= north_west)
		{
			// west direction
			kick_start_position = get_straight_kick_position_right();
		}
	}
	else if (angle_between_robot_and_ball > south_east || angle_between_robot_and_ball <= south_west)
	{
		// south sector
		if (angle_between_ball_and_goal > north_west || angle_between_ball_and_goal <= north_east)
		{
			// north direction
			kick_start_position = get_straight_kick_position_back();
		}
		else if (angle_between_ball_and_goal > north_east || angle_between_ball_and_goal <= south_east)
		{
			// east direction
			kick_start_position = get_side_kick_position_back_left();
		}
		else if (angle_between_ball_and_goal > south_east || angle_between_ball_and_goal <= south_west)
		{
			// south direction
			kick_start_position = get_straight_kick_position_front();
		}
		else if (angle_between_ball_and_goal > south_west || angle_between_ball_and_goal <= north_west)
		{
			// west direction
			kick_start_position = get_side_kick_position_back_right();
		}
	}
	else if (angle_between_robot_and_ball > south_west || angle_between_robot_and_ball <= north_west)
	{
		// west sector
		if (angle_between_ball_and_goal > north_west || angle_between_ball_and_goal <= north_east)
		{
			// north direction
			kick_start_position = get_side_kick_position_back_left();
		}
		else if (angle_between_ball_and_goal > north_east || angle_between_ball_and_goal <= south_east)
		{
			// east direction
			kick_start_position = get_straight_kick_position_left();
		}
		else if (angle_between_ball_and_goal > south_east || angle_between_ball_and_goal <= south_west)
		{
			// south direction
			kick_start_position = get_side_kick_position_front_left();
		}
		else if (angle_between_ball_and_goal > south_west || angle_between_ball_and_goal <= north_west)
		{
			// west direction
			kick_start_position = get_default_kick_position();
		}
	}

	return kick_start_position;
}

bool KickFactoryService::check_kicking_with_right(double angle_between_robot_and_ball)
{
	//TODO: make dynamic (adjustable via KickEngineParameter and/or Kick)
	//TODO: testing
	//TODO: cleanup

	return angle_between_robot_and_ball < 180;
}

struct3d KickFactoryService::get_default_kick_position()
{
	struct3d kick_position;

	// TODO: Replace placeholder-values
	kick_position->x = 0;
	kick_position->y = 0;
	kick_position->z = 0;

	return kick_position;
}

struct3d KickFactoryService::get_straight_kick_position_front()
{
	struct3d kick_position;

	// TODO: Replace placeholder-values
	kick_position->x = 0;
	kick_position->y = 10;
	kick_position->z = 5;

	return kick_position;
}

struct3d KickFactoryService::get_straight_kick_position_back()
{
	struct3d kick_position;

	// TODO: Replace placeholder-values
	kick_position->x = 0;
	kick_position->y = -10;
	kick_position->z = 10;

	return kick_position;
}

struct3d KickFactoryService::get_straight_kick_position_left()
{
	struct3d kick_position;

	// TODO: Replace placeholder-values
	kick_position->x = -10;
	kick_position->y = 0;
	kick_position->z = 10;

	return kick_position;
}

struct3d KickFactoryService::get_straight_kick_position_right()
{
	struct3d kick_position;

	// TODO: Replace placeholder-values
	kick_position->x = 10;
	kick_position->y = 0;
	kick_position->z = 10;

	return kick_position;
}

struct3d KickFactoryService::get_side_kick_position_front_left()
{
	struct3d kick_position;

	// TODO: Replace placeholder-values
	kick_position->x = -5;
	kick_position->y = 5;
	kick_position->z = 0;

	return kick_position;
}

struct3d KickFactoryService::get_side_kick_position_back_left()
{
	struct3d kick_position;

	// TODO: Replace placeholder-values
	kick_position->x = -5;
	kick_position->y = -5;
	kick_position->z = 0;

	return kick_position;
}

struct3d KickFactoryService::get_side_kick_position_front_right()
{
	struct3d kick_position;

	// TODO: Replace placeholder-values
	kick_position->x = 5;
	kick_position->y = -5;
	kick_position->z = 0;

	return kick_position;
}

struct3d KickFactoryService::get_side_kick_position_back_right()
{
	struct3d kick_position;

	// TODO: Replace placeholder-values
	kick_position->x = 5;
	kick_position->y = 5;
	kick_position->z = 0;

	return kick_position;
}
