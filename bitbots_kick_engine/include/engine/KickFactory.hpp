#ifndef KICKFACADE_HPP
#define KICKFACADE_HPP

#include "../kicks/Kick.hpp"

class KickFactory
{
public:
	virtual bitbots_splines::SplineContainer make_kick_trajection(struct3d & const ball, struct3d & const goal);

private:
	void init(struct3d& const ball, struct3d& const goal);
	void reset();

	double calculate_angle(double const x, double const y);
	struct3d calculate_kick_start();
	struct3d get_foot_position();
	bool check_kicking_with_left();

	virtual bitbots_splines::SplineContainer make_kick_trajection();

	struct3d const* m_pc_current_foot_position;
	struct3d const* m_pc_kick_start_foot_position;
	struct3d const* m_pc_current_ball_position;
	struct3d const* m_pc_ball_goal_position;
	bool m_b_kick_with_left;

	double m_d_angle_ball_goal;
	double m_d_angle_robot_ball;
};

#endif