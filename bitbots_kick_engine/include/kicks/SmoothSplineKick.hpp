#ifndef SMOOTHSPLINEKICK_HPP
#define SMOOTHSPLINEKICK_HPP

#include "Kick.hpp"
#include "../../../bitbots_spline/include/spline/SmoothSpline.hpp"

class SmoothSplineKick : public Kick
{
public:

protected:
	virtual bool check_requirements(KickParameter& kick_parameter) override;
	virtual bitbots_splines::SplineContainer init_trajectories() override;
	virtual bool calculate_movement_kick_preparation(double& time, struct3d const& foot_position, struct3d const& kick_start_position, bool const & kick_with_right) override;
	virtual void calculate_movement_kick(double& time, struct3d const& foot_position, struct3d const& ball_position, struct3d const& const kick_goal_position, bool const& kick_with_right) override;
	virtual void calculate_movement_kick_conclusion(double& time, struct3d const& foot_position, struct3d const& foot_ending_position, bool const& kick_with_right) override;


private:
};

#endif