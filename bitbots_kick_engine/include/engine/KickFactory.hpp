#ifndef KICKFACADE_HPP
#define KICKFACADE_HPP

#include "../kicks/Kick.hpp"
#include <memory>

class KickFactory
{
public:
	virtual bitbots_splines::SplineContainer* make_kick_trajection(struct3d * ball_position, struct3d * goal_position);
	virtual bitbots_splines::SplineContainer* make_kick_trajection(struct3d * current_foot_position, struct3d * ball_position, struct3d * goal_position);
	virtual bitbots_splines::SplineContainer* make_kick_trajection(struct3d * current_foot_position, struct3d * ball_position, struct3d * goal_position, struct3d * final_foot_position);

private:
	void init(struct3d * current_foot_position, struct3d * ball_position, struct3d * goal_position, struct3d * final_foot_position);
	void reset();

	double calculate_angle(double const x, double const y);
	struct3d * calculate_kick_start();
	bool check_kicking_with_right();

	Kick * create_kick();

	std::shared_ptr<struct3d> m_sp_current_foot_position;
	std::shared_ptr<struct3d> m_sp_kick_start_foot_position;
	std::shared_ptr<struct3d> m_sp_current_ball_position;
	std::shared_ptr<struct3d> m_sp_ball_goal_position;
	std::shared_ptr<struct3d> m_sp_final_foot_position;
	bool m_b_kick_with_right;

	double m_d_angle_ball_goal;
	double m_d_angle_robot_ball;
};

#endif