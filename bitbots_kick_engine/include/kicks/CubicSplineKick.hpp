#ifndef CUBICSPLINEKICK_HPP
#define CUBICSPLINEKICK_HPP

#include "Kick.hpp"

class CubicSplineKick : public Kick
{
public:
	CubicSplineKick(std::shared_ptr<KickEngineParameter> sp_parameter);

protected:
	virtual bool checkRequirements(KickAttributes & kick_attributes) override;
	virtual bool calculateMovementKickPreparation(double& time, struct3d & foot_position, struct3d & kick_start_position, bool & kick_with_right) override;
	virtual void calculateMovementKick(double& time, struct3d & foot_position, struct3d & ball_position, struct3d &  kick_goal_position, bool & kick_with_right) override;
	virtual void calculateMovementKickConclusion(double& time, struct3d & foot_position, struct3d & foot_ending_position, bool & kick_with_right) override;

private:
};

#endif