#ifndef KICKFACADE_HPP
#define KICKFACADE_HPP

#include "../kicks/Kick.hpp"

class KickFactory
{
public:
	virtual bitbots_splines::SplineContainer make_kick_trajection(struct3d & const ball, struct3d & const goal);

private:
	struct3d calculate_kick_start(struct3d & const ball, struct3d& const goal);
	struct3d get_foot_position(bool const kick_with_left);
	bool check_kicking_with_left(struct3d & const kick_start, struct3d & const ball);

	virtual bitbots_splines::SplineContainer make_kick_trajection(struct3d& const foot_current, struct3d& const foot_kick_start, struct3d& const ball_current, struct3d& const ball_goal, bool& const kick_with_left);
};

#endif