#ifndef KICK_HPP
#define KICK_HPP

#include <SplineContainer.hpp>
#include <memory>
#include <KickEngineParameter.hpp>
#include <KickParameter.hpp>

class Kick
{
public:
	Kick(std::shared_ptr<KickEngineParameter> sp_parameter);

	std::shared_ptr<bitbots_splines::SplineContainer> create_trajectories(KickParameter & kick_parameter);
	
	virtual bool can_execute_kick(KickParameter kick_parameter) = 0;

protected:
	virtual bool check_requirements(KickParameter & kick_parameter) = 0;
	virtual bitbots_splines::SplineContainer init_trajectories() = 0;
	virtual bool calculate_movement_kick_preparation(double& time, struct3d const& const foot_position, struct3d const& const kick_start_position, bool const& const kick_with_right) = 0;
	virtual void calculate_movement_kick(double& time, struct3d const& const foot_position, struct3d const& const ball_position, struct3d const& const kick_goal_position, bool const& const kick_with_right) = 0;
	virtual void calculate_movement_kick_conclusion(double& time, struct3d const& const foot_position, struct3d const& const foot_ending_position, bool const& const kick_with_right) = 0;

	bool check_foot_position_with_double_support(struct3d const& const foot_starting_position);
	void point(bitbots_splines::CurvePurpose spline_purpose, double time, double position, double velocity = 0, double acceleration = 0);

	std::shared_ptr<KickEngineParameter> m_sp_parameter;

private:
	std::shared_ptr<bitbots_splines::SplineContainer> m_sp_spline_container;
};

#endif