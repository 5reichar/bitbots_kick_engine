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
	kick(struct3d const foot_current, struct3d const foot_kick_start, struct3d const ball_current, struct3d const ball_goal, bool const kick_with_left_foot);

	bitbots_splines::SplineContainer & create_trajectory();

protected:
	virtual void calculate_kick() = 0;

	struct3d const foot_current;
	struct3d const foot_kick_start;
	struct3d const ball_current;
	struct3d const ball_goal;
	bool const kick_with_left_feet;

private:

};

#endif