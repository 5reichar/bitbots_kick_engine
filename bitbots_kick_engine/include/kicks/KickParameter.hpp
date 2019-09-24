#ifndef KICKPARAMTER_HPP
#define KICKPARAMTER_HPP

#include <memory>

struct struct3d
{
	double x, y, z;
};

class KickParameter
{
public:
	KickParameter();

	void reset();

	void set_foot_starting_position(std::shared_ptr<struct3d> position);
	void set_foot_position_for_kick(std::shared_ptr<struct3d> position);
	void set_ball_position(std::shared_ptr<struct3d> position);
	void set_kick_goal_position(std::shared_ptr<struct3d> position);
	void set_foot_ending_position(std::shared_ptr<struct3d> position);

	void set_kick_with_right(bool kick_with_right);

	bool has_kick_prepareration();
	bool has_kick_starting_position();
	bool has_kick_ball_postion();
	bool has_kick_goal();
	bool has_kick_conclusion();

	bool kick_with_right();

	std::shared_ptr<struct3d> get_foot_starting_position();
	std::shared_ptr<struct3d> get_foot_position_for_kick();
	std::shared_ptr<struct3d> get_ball_position();
	std::shared_ptr<struct3d> get_kick_goal_position();
	std::shared_ptr<struct3d> get_foot_ending_position();

private:
	std::shared_ptr<struct3d> m_sp_foot_starting_position;
	std::shared_ptr<struct3d> m_sp_foot_position_for_kick;
	std::shared_ptr<struct3d> m_sp_ball_position;
	std::shared_ptr<struct3d> m_sp_kick_goal_position;
	std::shared_ptr<struct3d> m_sp_foot_ending_position;

	bool m_b_kick_with_right;
};

#endif