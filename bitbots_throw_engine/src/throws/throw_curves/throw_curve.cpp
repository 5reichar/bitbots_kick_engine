#include "throws/throw_curves/throw_curve.h"

bool ThrowCurve::calculate_trajectories(std::shared_ptr<ThrowParameter> & throw_parameter)
{
	//TODO: testing
	//TODO: cleanup

    bool b_succ = false;

	//checks
	if (check_requirements(throw_parameter))
	{
        // build and return spline container
        double d_time = 0.0;

        calculate_pick_up_ball_movement(d_time, throw_parameter); // Optional
        calculate_throw_movement(d_time, throw_parameter);
        calculate_throw_conclusion_movement(d_time, throw_parameter); // Optional
	}

    // implement ROS Error

    return b_succ;
}

bool ThrowCurve::check_requirements(std::shared_ptr<ThrowParameter> & throw_parameter)
{
	return true;
}

std::shared_ptr<bitbots_splines::SplineContainer> ThrowCurve::get_spline_container()const
{
	//TODO: testing
	//TODO: cleanup

    return sp_spline_container_;
}

void ThrowCurve::calculate_pick_up_ball_movement(double & time, std::shared_ptr<ThrowParameter> & throw_parameter)
{
    // Empty function, so Children class can implement if the need it

    return;
}

void ThrowCurve::calculate_throw_movement(double & time, std::shared_ptr<ThrowParameter> & throw_parameter)
{
    //TODO: rework of parameter
	//TODO: testing
	//TODO: cleanup

	/////  Preparation
	//Set up the trajectories for the half cycle (single step)
	double start_time = time;
	double pick_up_ball_time = time + 1.0 / (4.0 * throw_parameter->movement_cycle_frequence);
	double begin_throw_time = time + 2.0 / (4.0 * throw_parameter->movement_cycle_frequence);
	double release_throw_time = time + 3.0 / (4.0 * throw_parameter->movement_cycle_frequence);
	double finish_time = time + throw_parameter->movement_cycle_frequence;

	/////  Left Hand Position
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_x, start_time, throw_parameter->left_hand_start_position.x);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_x, pick_up_ball_time, throw_parameter->ball_position.x);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_x, begin_throw_time, throw_parameter->throw_start_position.x);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_x, release_throw_time, throw_parameter->throw_release_position.x, throw_parameter->throw_velocity);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_x, finish_time, throw_parameter->left_hand_end_position.x);

	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_y, start_time, throw_parameter->left_hand_start_position.y);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_y, pick_up_ball_time, throw_parameter->ball_position.y);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_y, begin_throw_time, throw_parameter->throw_start_position.y);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_y, release_throw_time, throw_parameter->throw_release_position.y, throw_parameter->throw_velocity);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_y, finish_time, throw_parameter->left_hand_end_position.y);

	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_z, start_time, throw_parameter->left_hand_start_position.z);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_z, pick_up_ball_time, throw_parameter->ball_position.z);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_z, begin_throw_time, throw_parameter->throw_start_position.z);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_z, release_throw_time, throw_parameter->throw_release_position.z, throw_parameter->throw_velocity);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_z, finish_time, throw_parameter->left_hand_end_position.z);

	/////  Left Hand Axis
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_x, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_x, pick_up_ball_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_x, begin_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_x, release_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_x, finish_time, 0.0);

	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_y, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_y, pick_up_ball_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_y, begin_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_y, release_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_y, finish_time, 0.0);

	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_z, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_z, pick_up_ball_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_z, begin_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_z, release_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_z, finish_time, 0.0);

	/////  Right Hand Position
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_x, start_time, throw_parameter->right_hand_start_position.x);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_x, pick_up_ball_time, throw_parameter->ball_position.x);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_x, begin_throw_time, throw_parameter->throw_start_position.x);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_x, release_throw_time, throw_parameter->throw_release_position.x, throw_parameter->throw_velocity);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_x, finish_time, throw_parameter->right_hand_end_position.x);

	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_y, start_time, throw_parameter->right_hand_start_position.y);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_y, pick_up_ball_time, throw_parameter->ball_position.y);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_y, begin_throw_time, throw_parameter->throw_start_position.y);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_y, release_throw_time, throw_parameter->throw_release_position.y, throw_parameter->throw_velocity);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_y, finish_time, throw_parameter->right_hand_end_position.y);

	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_z, start_time, throw_parameter->right_hand_start_position.z);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_z, pick_up_ball_time, throw_parameter->ball_position.z);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_z, begin_throw_time, throw_parameter->throw_start_position.z);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_z, release_throw_time, throw_parameter->throw_release_position.z, throw_parameter->throw_velocity);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_z, finish_time, throw_parameter->right_hand_end_position.z);

	/////  Right Hand Axis
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_x, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_x, pick_up_ball_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_x, begin_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_x, release_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_x, finish_time, 0.0);

	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_y, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_y, pick_up_ball_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_y, begin_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_y, release_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_y, finish_time, 0.0);

	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_z, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_z, pick_up_ball_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_z, begin_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_z, release_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_z, finish_time, 0.0);

	/////  Trunk Position
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_x, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_x, pick_up_ball_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_x, begin_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_x, release_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_x, finish_time, 0.0);

	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_y, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_y, pick_up_ball_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_y, begin_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_y, release_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_y, finish_time, 0.0);

	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_z, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_z, pick_up_ball_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_z, begin_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_z, release_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_z, finish_time, 0.0);

	/////  Trunk Axis
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_x, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_x, pick_up_ball_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_x, begin_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_x, release_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_x, finish_time, 0.0);

	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_y, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_y, pick_up_ball_time, throw_parameter->pick_up_bow_angle);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_y, begin_throw_time, throw_parameter->throw_start_pitch);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_y, release_throw_time, throw_parameter->throw_release_pitch);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_y, finish_time, 0.0);

	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_z, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_z, pick_up_ball_time, throw_parameter->pick_up_orientation);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_z, begin_throw_time, throw_parameter->throw_orientation);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_z, release_throw_time, throw_parameter->throw_orientation);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_z, finish_time, 0.0);

	/////  Clean Up
	time = finish_time;
}

void ThrowCurve::calculate_throw_conclusion_movement(double & time, std::shared_ptr<ThrowParameter> & throw_parameter)
{
    // Empty function, so Children class can implement if the need it

    return;
}

void ThrowCurve::add_point_to_spline(bitbots_splines::CurvePurpose spline_purpose, double time, double position, double velocity, double acceleration)
{
	//TODO: testing
	//TODO: cleanup

	sp_spline_container_->get(spline_purpose)->addPoint(time, position, velocity, acceleration);
}
