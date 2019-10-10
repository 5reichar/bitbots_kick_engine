#ifndef KICKFACADE_HPP
#define KICKFACADE_HPP

#include <memory>
#include "KickEngineParameter.hpp"
#include "../kicks/Kick.hpp"
#include "../kicks/KickParameter.hpp"

class KickFactory
{
public:
	KickFactory(std::shared_ptr<KickEngineParameter> sp_parameter);

	virtual bitbots_splines::SplineContainer* make_kick_trajection(struct3d * ball_position, struct3d * goal_position);
	virtual bitbots_splines::SplineContainer* make_kick_trajection(struct3d * current_foot_position, struct3d * ball_position, struct3d * goal_position, struct3d * final_foot_position);

private:
	KickParameter init(struct3d * current_foot_position, struct3d * ball_position, struct3d * goal_position, struct3d * final_foot_position);

	double calculate_angle(double const x, double const y);
	struct3d * calculate_kick_start(double angle_between_robot_and_ball, double angle_between_ball_and_goal);
	void set_default_kick_position(struct3d* kick_position);
	void set_straight_kick_position_front(struct3d* kick_position);
	void set_straight_kick_position_back(struct3d* kick_position);
	void set_straight_kick_position_left(struct3d* kick_position);
	void set_straight_kick_position_right(struct3d* kick_position);
	void set_side_kick_position_front_left(struct3d* kick_position);
	void set_side_kick_position_back_left(struct3d* kick_position);
	void set_side_kick_position_front_right(struct3d* kick_position);
	void set_side_kick_position_back_right(struct3d* kick_position);
	bool check_kicking_with_right(double angle_between_robot_and_ball);
	bool check_generale_requirements(KickParameter & kick_parameter);

	std::shared_ptr<Kick> create_kick();

	std::shared_ptr<KickEngineParameter> m_sp_parameter;

	std::shared_ptr<struct3d> m_sp_current_foot_position;
	std::shared_ptr<struct3d> m_sp_kick_start_foot_position;
	std::shared_ptr<struct3d> m_sp_current_ball_position;
	std::shared_ptr<struct3d> m_sp_ball_goal_position;
	std::shared_ptr<struct3d> m_sp_final_foot_position;
	bool m_b_kick_with_right;

	double m_d_angle_ball_goal;
	double m_d_angle_robot_ball;

	const double north;
	const double north_east;
	const double east;
	const double south_east;
	const double south;
	const double south_west;
	const double west;
	const double north_west;
};

#endif