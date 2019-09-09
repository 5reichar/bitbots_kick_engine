#ifndef KICK_HPP
#define KICK_HPP

#include "../../bitbots_spline/include/utils/SplineContainer.hpp"
#include <memory>

struct struct3d
{
	double x, y, z;
};

class Kick
{
public:
	void reset();
	std::shared_ptr<bitbots_splines::SplineContainer> create_trajectories();

	void set_foot_position(std::shared_ptr<struct3d> position);
	void set_kick_start_position(std::shared_ptr<struct3d> position);
	void set_ball_position(std::shared_ptr<struct3d> position);
	void set_goal_position(std::shared_ptr<struct3d> position);
	void set_foot_end_position(std::shared_ptr<struct3d> position);
	void set_kick_with_right(bool kick_with_right);

protected:
	virtual bool additional_requirements();
	virtual void build_trajectories() = 0;

	void point(bitbots_splines::CurvePurpose spline_purpose, double time, double position, double velocity = 0, double acceleration = 0);

	std::shared_ptr<struct3d> m_sp_foot_position;
	std::shared_ptr<struct3d> m_sp_kick_start_position;
	std::shared_ptr<struct3d> m_sp_ball_position;
	std::shared_ptr<struct3d> m_sp_goal_position;
	std::shared_ptr<struct3d> m_sp_foot_end_position;
	bool m_b_kick_with_right;

private:
	std::shared_ptr<bitbots_splines::SplineContainer> m_sp_spline_container;
};

#endif