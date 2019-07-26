#ifndef KICK_HPP
#define KICK_HPP

#include <bitbots_spline/include/utils/SplineContainer.hpp>

struct struct3d
{
	double x, y, z;
};

class Kick
{
public:
	bitbots_splines::SplineContainer & create_trajectory(struct3d const foot_current, struct3d const foot_kick_start, struct3d const ball_current, struct3d const ball_goal, bool const kick_with_left_foot);

protected:
	virtual bitbots_splines::SplineContainer calculate_kick(struct3d const foot_current, struct3d const foot_kick_start, struct3d const ball_current, struct3d const ball_goal) = 0;
	virtual void set_support_foot_trajectories(bitbots_splines::SplineContainer& spline_container, bool const kick_with_left_foot) = 0;

private:
};

#endif