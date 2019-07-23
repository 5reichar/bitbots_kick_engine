#include "KickFactory.hpp"

bitbots_splines::SplineContainer KickFactory::make_kick_trajection(struct3d & const ball, struct3d & const goal)
{
	auto kick_start = calculate_kick_start(ball, goal);
	auto kick_with_left_foot = check_kicking_with_left(kick_start, ball);
	auto kick_foot = get_foot_position(kick_with_left_foot);

	return make_kick_trajection(kick_foot, kick_start, ball, goal, kick_with_left_foot);
}

struct3d KickFactory::calculate_kick_start(struct3d & const ball, struct3d & const goal)
{
	return struct3d();
}

struct3d KickFactory::get_foot_position(bool const kick_with_left)
{
	return struct3d();
}

bool KickFactory::check_kicking_with_left(struct3d & const kick_start, struct3d & const ball)
{
	return false;
}

bitbots_splines::SplineContainer KickFactory::make_kick_trajection(struct3d & const foot_current, struct3d & const foot_kick_start, struct3d & const ball_current, struct3d & const ball_goal, bool const kick_with_left)
{
	return bitbots_splines::SplineContainer();
}
