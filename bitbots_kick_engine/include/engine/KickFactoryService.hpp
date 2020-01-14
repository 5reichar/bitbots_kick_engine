#ifndef KICKFACTORYSERVICE_HPP
#define KICKFACTORYSERVICE_HPP


#include "KickEngineParameter.hpp"
#include "../kicks/KickParameter.hpp"

class KickFactoryService
{
public:
	static double calculate_angle(double const x, double const y);
	static bool check_kicking_with_right(double angle_between_robot_and_ball);
	static bool check_angle_requirements(double angle, AngleRequirements requirements);

	static struct3d calculate_kick_start(double angle_between_robot_and_ball, double angle_between_ball_and_goal);

private:
	static struct3d get_default_kick_position();
	static struct3d get_straight_kick_position_front();
	static struct3d get_straight_kick_position_back();
	static struct3d get_straight_kick_position_left();
	static struct3d get_straight_kick_position_right();
	static struct3d get_side_kick_position_front_left();
	static struct3d get_side_kick_position_back_left();
	static struct3d get_side_kick_position_front_right();
	static struct3d get_side_kick_position_back_right();

	static const double north;
	static const double north_east;
	static const double east;
	static const double south_east;
	static const double south;
	static const double south_west;
	static const double west;
	static const double north_west;
};

#endif