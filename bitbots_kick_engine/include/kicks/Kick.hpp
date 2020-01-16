#ifndef KICK_HPP
#define KICK_HPP

#include <memory>
#include "KickParameter.hpp"
#include "../engine/KickEngineParameter.hpp"
#include "../../../bitbots_spline/include/utils/SplineContainer.hpp"

class Kick
{
public:
	Kick(std::shared_ptr<KickEngineParameter> sp_parameter);

	std::shared_ptr<bitbots_splines::SplineContainer> calculate_trajectories(KickAttributes & kick_attributes);

	//virtual bool can_execute_kick(KickAttributes & kick_attributes) = 0;
	virtual bool use_default_calculation_for_kick_stating_position();

protected:
	virtual bool check_requirements(KickAttributes & kick_attributes) = 0;
	virtual bool calculate_movement_kick_preparation(double& time, struct3d & foot_position, struct3d & kick_start_position, bool & kick_with_right) = 0;
	virtual void calculate_movement_kick(double& time, struct3d & foot_position, struct3d & ball_position, struct3d & kick_goal_position, bool & kick_with_right) = 0;
	virtual void calculate_movement_kick_conclusion(double& time, struct3d & foot_position, struct3d & foot_ending_position, bool & kick_with_right) = 0;

	bool check_foot_position_with_double_support(struct3d & foot_starting_position);
	void point(bitbots_splines::CurvePurpose spline_purpose, double time, double position, double velocity = 0, double acceleration = 0);

	std::shared_ptr<KickEngineParameter> m_sp_parameter;
	std::shared_ptr<bitbots_splines::SplineContainer> m_sp_spline_container;
};

#endif