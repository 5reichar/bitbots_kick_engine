#include "kicks/SmoothSplineKick.hpp"
#include "../../bitbots_spline/include/utils/TrajectoryService.hpp"

SmoothSplineKick::SmoothSplineKick(std::shared_ptr<KickEngineParameter> sp_parameter)
	:Kick(sp_parameter)
{

}

bool SmoothSplineKick::check_requirements(KickAttributes & kick_attributes)
{
	//TODO: testing
	//TODO: cleanup

	return true;
}

bitbots_splines::SplineContainer SmoothSplineKick::init_trajectories()
{
	//TODO: testing
	//TODO: cleanup

	return bitbots_splines::TrajectoryService::TrajectoriesInit<bitbots_splines::SmoothSpline>();
}

bool SmoothSplineKick::calculate_movement_kick_preparation(double& time, struct3d &  foot_position, struct3d &  kick_start_position, bool &  kick_with_right)
{
	//TODO: testing
	//TODO: cleanup

	//Set up the trajectories for the half cycle (single step)
	double halfPeriod = 1.0 / (2.0 * m_sp_parameter->freq);
	// full period (double step) is needed for trunk splines
	double period = 2.0 * halfPeriod;


	point(bitbots_splines::CurvePurpose::is_double_support, time, check_foot_position_with_double_support(foot_position));
	point(bitbots_splines::CurvePurpose::is_double_support, time + halfPeriod, 0.0);
	point(bitbots_splines::CurvePurpose::is_double_support, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::is_left_support_foot, time, kick_with_right);
	point(bitbots_splines::CurvePurpose::is_left_support_foot, time + halfPeriod, kick_with_right);
	point(bitbots_splines::CurvePurpose::is_left_support_foot, time + period, kick_with_right);

	//TODO: Test and rework
	point(bitbots_splines::CurvePurpose::trunk_position_x, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_position_x, time + period, kick_start_position.x * -0.5);
	
	//TODO: Test and rework
	point(bitbots_splines::CurvePurpose::trunk_position_y, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_position_y, time + period, kick_start_position.y * -0.5);

	point(bitbots_splines::CurvePurpose::trunk_position_z, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_position_z, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::trunk_axis_x, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_axis_x, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::trunk_axis_y, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_axis_y, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::trunk_axis_z, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_axis_z, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::foot_position_x, time, foot_position.x);
	point(bitbots_splines::CurvePurpose::foot_position_x, time + period, kick_start_position.x);

	point(bitbots_splines::CurvePurpose::foot_position_y, time, foot_position.y);
	point(bitbots_splines::CurvePurpose::foot_position_y, time + period, kick_start_position.y);

	point(bitbots_splines::CurvePurpose::foot_position_z, time, foot_position.z);
	point(bitbots_splines::CurvePurpose::foot_position_z, time + period, kick_start_position.z);

	point(bitbots_splines::CurvePurpose::foot_axis_x, time, 0.0);
	point(bitbots_splines::CurvePurpose::foot_axis_x, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::foot_axis_y, time, 0.0);
	point(bitbots_splines::CurvePurpose::foot_axis_y, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::foot_axis_z, time, 0.0);
	point(bitbots_splines::CurvePurpose::foot_axis_z, time + halfPeriod, 0.0);
	point(bitbots_splines::CurvePurpose::foot_axis_z, time + period, 0.0);

	time += period;
	return true;
}

void SmoothSplineKick::calculate_movement_kick(double& time, struct3d &  foot_position, struct3d &  ball_position, struct3d &  kick_goal_position, bool &  kick_with_right)
{
	//TODO: testing
	//TODO: cleanup

	//Set up the trajectories for the half cycle (single step)
	double halfPeriod = 1.0 / (2.0 * m_sp_parameter->freq);
	// full period (double step) is needed for trunk splines
	double period = 2.0 * halfPeriod;


	point(bitbots_splines::CurvePurpose::is_double_support, time, check_foot_position_with_double_support(foot_position));
	point(bitbots_splines::CurvePurpose::is_double_support, time + halfPeriod, 0.0);
	point(bitbots_splines::CurvePurpose::is_double_support, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::is_left_support_foot, time, kick_with_right);
	point(bitbots_splines::CurvePurpose::is_left_support_foot, time + halfPeriod, kick_with_right);
	point(bitbots_splines::CurvePurpose::is_left_support_foot, time + period, kick_with_right);

	//TODO: Test and rework
	point(bitbots_splines::CurvePurpose::trunk_position_x, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_position_x, time + period, ball_position.x * -0.5);

	//TODO: Test and rework
	point(bitbots_splines::CurvePurpose::trunk_position_y, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_position_y, time + period, ball_position.y * -0.5);

	point(bitbots_splines::CurvePurpose::trunk_position_z, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_position_z, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::trunk_axis_x, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_axis_x, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::trunk_axis_y, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_axis_y, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::trunk_axis_z, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_axis_z, time + period, 0.0);

	//TODO: Test and rework
	point(bitbots_splines::CurvePurpose::foot_position_x, time, foot_position.x);
	point(bitbots_splines::CurvePurpose::foot_position_x, time + period, ball_position.x, std::abs(kick_goal_position.x - ball_position.x) / period);

	//TODO: Test and rework
	point(bitbots_splines::CurvePurpose::foot_position_y, time, foot_position.y);
	point(bitbots_splines::CurvePurpose::foot_position_y, time + period, ball_position.y, std::abs(kick_goal_position.y - ball_position.y) / period);

	point(bitbots_splines::CurvePurpose::foot_position_z, time, foot_position.z);
	point(bitbots_splines::CurvePurpose::foot_position_z, time + period, ball_position.z);

	point(bitbots_splines::CurvePurpose::foot_axis_x, time, 0.0);
	point(bitbots_splines::CurvePurpose::foot_axis_x, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::foot_axis_y, time, 0.0);
	point(bitbots_splines::CurvePurpose::foot_axis_y, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::foot_axis_z, time, 0.0);
	point(bitbots_splines::CurvePurpose::foot_axis_z, time + halfPeriod, 0.0);
	point(bitbots_splines::CurvePurpose::foot_axis_z, time + period, 0.0);

	time += period;
}

void SmoothSplineKick::calculate_movement_kick_conclusion(double& time, struct3d &  foot_position, struct3d &  foot_ending_position, bool &  kick_with_right)
{
	//TODO: testing
	//TODO: cleanup

	//Set up the trajectories for the half cycle (single step)
	double halfPeriod = 1.0 / (2.0 * m_sp_parameter->freq);
	// full period (double step) is needed for trunk splines
	double period = 2.0 * halfPeriod;


	point(bitbots_splines::CurvePurpose::is_double_support, time, check_foot_position_with_double_support(foot_position));
	point(bitbots_splines::CurvePurpose::is_double_support, time + halfPeriod, 0.0);
	point(bitbots_splines::CurvePurpose::is_double_support, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::is_left_support_foot, time, kick_with_right);
	point(bitbots_splines::CurvePurpose::is_left_support_foot, time + halfPeriod, kick_with_right);
	point(bitbots_splines::CurvePurpose::is_left_support_foot, time + period, kick_with_right);

	//TODO: Test and rework
	point(bitbots_splines::CurvePurpose::trunk_position_x, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_position_x, time + period, foot_ending_position.x * -0.5);

	//TODO: Test and rework
	point(bitbots_splines::CurvePurpose::trunk_position_y, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_position_y, time + period, foot_ending_position.y * -0.5);

	point(bitbots_splines::CurvePurpose::trunk_position_z, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_position_z, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::trunk_axis_x, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_axis_x, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::trunk_axis_y, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_axis_y, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::trunk_axis_z, time, 0.0);
	point(bitbots_splines::CurvePurpose::trunk_axis_z, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::foot_position_x, time, foot_position.x);
	point(bitbots_splines::CurvePurpose::foot_position_x, time + period, foot_ending_position.x);

	point(bitbots_splines::CurvePurpose::foot_position_y, time, foot_position.y);
	point(bitbots_splines::CurvePurpose::foot_position_y, time + period, foot_ending_position.y);

	point(bitbots_splines::CurvePurpose::foot_position_z, time, foot_position.z);
	point(bitbots_splines::CurvePurpose::foot_position_z, time + period, foot_ending_position.z);

	point(bitbots_splines::CurvePurpose::foot_axis_x, time, 0.0);
	point(bitbots_splines::CurvePurpose::foot_axis_x, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::foot_axis_y, time, 0.0);
	point(bitbots_splines::CurvePurpose::foot_axis_y, time + period, 0.0);

	point(bitbots_splines::CurvePurpose::foot_axis_z, time, 0.0);
	point(bitbots_splines::CurvePurpose::foot_axis_z, time + halfPeriod, 0.0);
	point(bitbots_splines::CurvePurpose::foot_axis_z, time + period, 0.0);

	time += period;
}