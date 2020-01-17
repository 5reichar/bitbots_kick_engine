#ifndef KICKFACTORYSERVICE_HPP
#define KICKFACTORYSERVICE_HPP


#include "KickEngineParameter.hpp"
#include "../kicks/KickParameter.hpp"
#include <memory>

class KickFactoryService
{
public:
	static double calculate_angle(double const x, double const y);
	static bool check_kicking_with_right(double angle_between_robot_and_ball);
	static bool check_angle_requirements(double angle, AngleRequirements requirements);
	static struct3d get_kick_preparation_position(double angle_between_robot_and_ball, double angle_between_ball_and_goal, std::shared_ptr<KickParameter> params);

private:

};

#endif