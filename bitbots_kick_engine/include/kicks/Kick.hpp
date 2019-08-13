#ifndef KICK_HPP
#define KICK_HPP

#include "../../bitbots_spline/include/utils/SplineContainer.hpp"

struct struct3d
{
	double x, y, z;
};

class Kick
{
public:
	void reset();
	bitbots_splines::SplineContainer & create_trajectories();

	void set_foot_position(std::shared_ptr<struct3d> position);
	void set_kick_start_position(std::shared_ptr<struct3d> position);
	void set_ball_position(std::shared_ptr<struct3d> position);
	void set_goal_position(std::shared_ptr<struct3d> position);
	void set_foot_end_position(std::shared_ptr<struct3d> position);
	void set_kick_with_right(bool kick_with_right);

protected:
	virtual bool additional_requirements();
	virtual bitbots_splines::SplineContainer calculate_trajectories() = 0;

	std::shared_ptr<struct3d> m_sp_foot_position;
	std::shared_ptr<struct3d> m_sp_kick_start_position;
	std::shared_ptr<struct3d> m_sp_ball_position;
	std::shared_ptr<struct3d> m_sp_goal_position;
	std::shared_ptr<struct3d> m_sp_foot_end_position;
	bool m_b_kick_with_right;

private:
};

#endif