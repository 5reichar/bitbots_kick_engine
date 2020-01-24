#ifndef KICKFACTORYSERVICE_HPP
#define KICKFACTORYSERVICE_HPP


#include "KickEngineParameter.hpp"
#include "../kicks/KickParameter.hpp"
#include <memory>

class KickFactoryService
{
public:
	static double calculateAngle(double const x, double const y);
	static bool checkKickingWithRight(double angle_between_robot_and_ball);
	static bool checkAngleRequirements(double angle, AngleRequirements requirements);
	static struct3d getKickPreparationPosition(double angle_between_robot_and_ball, double angle_between_ball_and_goal, std::shared_ptr<KickParameter> params);

private:

};

#endif