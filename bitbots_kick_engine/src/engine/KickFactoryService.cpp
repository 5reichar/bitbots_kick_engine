#include "engine/KickFactoryService.hpp"
#include <math.h>

double KickFactoryService::calculateAngle(double const x, double const y)
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

bool KickFactoryService::checkKickingWithRight(double angle_between_robot_and_ball)
{
	//TODO: make dynamic (adjustable via KickEngineParameter and/or Kick)
	//TODO: testing
	//TODO: cleanup

	return angle_between_robot_and_ball < 180;
}

bool KickFactoryService::checkAngleRequirements(double angle, AngleRequirements requirements)
{
	bool result = false;

	// check if we have a angle range like 300 to 50 degree
	if (requirements.min_angle > requirements.max_angle)
	{
		// if we have, only one of the ranges (e.g. 300 - 360 or 0 - 50)
		result = requirements.min_angle <= angle || angle <= requirements.max_angle;
	}
	else
	{
		result = requirements.min_angle <= angle && angle <= requirements.max_angle;
	}

	return result;
}

struct3d KickFactoryService::getKickPreparationPosition(double angle_between_robot_and_ball, double angle_between_ball_and_goal, std::shared_ptr<KickParameter> params)
{
	struct3d result = params->default_kick_preparation_position;
	
	for(auto it = params->v_kick_preparation_positions.begin(); it != params->v_kick_preparation_positions.end(); ++it)
	{
		if(checkAngleRequirements(angle_between_robot_and_ball, it->angle_requiremts_robot_ball)
		&& checkAngleRequirements(angle_between_ball_and_goal, it->angle_requiremts_ball_goal))
		{
			result = it->position;
			break;
		}
	}

	return result;
}