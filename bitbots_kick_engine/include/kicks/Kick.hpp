#ifndef KICK_HPP
#define KICK_HPP

#include <memory>
#include "KickParameter.hpp"
#include "../engine/KickEngineParameter.hpp"
#include "../../bitbots_spline/include/utils/spline_container_factory.hpp"

class Kick
{
public:
	Kick(std::shared_ptr<KickEngineParameter> sp_parameter);

	std::shared_ptr<bitbots_splines::SplineContainer> calculateTrajectories(KickAttributes & kick_attributes);

	virtual bool useDefaultCalculationForKickStatingPosition();

protected:
	virtual bool checkRequirements(KickAttributes & kick_attributes) = 0;
	virtual bool calculateMovementKickPreparation(double& time, struct3d & foot_position, struct3d & kick_start_position, bool & kick_with_right) = 0;
	virtual void calculateMovementKick(double& time, struct3d & foot_position, struct3d & ball_position, struct3d & kick_goal_position, bool & kick_with_right) = 0;
	virtual void calculateMovementKickConclusion(double& time, struct3d & foot_position, struct3d & foot_ending_position, bool & kick_with_right) = 0;

	bool checkFootPositionWithDoubleSupport(struct3d & foot_starting_position);
	void point(bitbots_splines::CurvePurpose spline_purpose, double time, double position, double velocity = 0, double acceleration = 0);

	std::shared_ptr<KickEngineParameter> m_sp_parameter;
	std::shared_ptr<bitbots_splines::SplineContainer> m_sp_spline_container;
};

#endif