#ifndef SMOOTHSPLINEKICK_HPP
#define SMOOTHSPLINEKICK_HPP

#include "Kick.hpp"
#include "../../../bitbots_spline/include/spline/SmoothSpline.hpp"

class SmoothSplineKick : public Kick
{
public:
	SmoothSplineKick(std::shared_ptr<KickEngineParameter> sp_parameter);

protected:
	virtual bool check_requirements(KickAttributes & kick_attributes) override;
	virtual bool calculate_movement_kick_preparation(double& time, struct3d & foot_position, struct3d & kick_start_position, bool & kick_with_right) override;
	virtual void calculate_movement_kick(double& time, struct3d & foot_position, struct3d & ball_position, struct3d &  kick_goal_position, bool & kick_with_right) override;
	virtual void calculate_movement_kick_conclusion(double& time, struct3d & foot_position, struct3d & foot_ending_position, bool & kick_with_right) override;

private:
};

#endif