#include "..\..\include\kicks\Kick.hpp"

bitbots_splines::SplineContainer& Kick::create_trajectory(struct3d const foot_current, struct3d const foot_kick_start, struct3d const ball_current, struct3d const ball_goal, bool const kick_with_left_foot)
{
	auto spline_container = calculate_kick(foot_current, foot_kick_start, ball_current, ball_goal);

	set_support_foot_trajectories(spline_container, kick_with_left_foot);

	return spline_container;
}