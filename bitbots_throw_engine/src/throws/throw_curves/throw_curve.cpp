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

        calculate_pick_up_ball_movement(d_time, throw_parameter);
        calculate_throw_movement(d_time, throw_parameter);
        calculate_throw_conclusion_movement(d_time, throw_parameter);

		b_succ = true;
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
    //TODO: rework of parameter
	//TODO: testing
	//TODO: cleanup

	/////  Preparation
	//Set up the trajectories for the half cycle (single step)
	double start_time = time;
	double pick_up_ball_time = time + (throw_parameter->pick_up_duration_share_ * throw_parameter->movement_cycle_frequence_);

	/////  Left Hand Position
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_x, start_time, throw_parameter->start_left_hand_position_.x_);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_x, pick_up_ball_time, throw_parameter->pick_up_left_hand_position_.x_);

	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_y, start_time, throw_parameter->start_left_hand_position_.y_);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_y, pick_up_ball_time, throw_parameter->pick_up_left_hand_position_.y_);

	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_z, start_time, throw_parameter->start_left_hand_position_.z_);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_z, pick_up_ball_time, throw_parameter->pick_up_left_hand_position_.z_);

	/////  Left Hand Axis
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_x, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_x, pick_up_ball_time, throw_parameter->pick_up_left_hand_axis_.x_);

	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_y, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_y, pick_up_ball_time, throw_parameter->pick_up_left_hand_axis_.y_);

	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_z, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_z, pick_up_ball_time, throw_parameter->pick_up_left_hand_axis_.z_);

	/////  Right Hand Position
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_x, start_time, throw_parameter->end_right_hand_position_.x_);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_x, pick_up_ball_time, throw_parameter->pick_up_right_hand_position_.x_);

	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_y, start_time, throw_parameter->end_right_hand_position_.y_);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_y, pick_up_ball_time, throw_parameter->pick_up_right_hand_position_.y_);

	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_z, start_time, throw_parameter->end_right_hand_position_.z_);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_z, pick_up_ball_time, throw_parameter->pick_up_right_hand_position_.z_);

	/////  Right Hand Axis
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_x, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_x, pick_up_ball_time, throw_parameter->pick_up_right_hand_axis_.x_);

	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_y, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_y, pick_up_ball_time, throw_parameter->pick_up_right_hand_axis_.y_);

	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_z, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_z, pick_up_ball_time, throw_parameter->pick_up_right_hand_axis_.z_);

	/////  Trunk Position
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_x, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_x, pick_up_ball_time, 0.0);

	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_y, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_y, pick_up_ball_time, 0.0);

	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_z, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_z, pick_up_ball_time, 0.0);

	/////  Trunk Axis
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_x, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_x, pick_up_ball_time, throw_parameter->pick_up_trunk_axis_.x_);

	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_y, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_y, pick_up_ball_time, throw_parameter->pick_up_trunk_axis_.y_); // Bow Angle

	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_z, start_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_z, pick_up_ball_time, throw_parameter->pick_up_trunk_axis_.z_); // Orientation

	/////  Clean Up
	time = pick_up_ball_time;
}

void ThrowCurve::calculate_throw_movement(double & time, std::shared_ptr<ThrowParameter> & throw_parameter)
{
    //TODO: rework of parameter
	//TODO: testing
	//TODO: cleanup

	/////  Preparation
	//Set up the trajectories for the half cycle (single step)
	double begin_throw_time = time + (throw_parameter->throw_preparation_duration_share_ * throw_parameter->movement_cycle_frequence_);
	double release_throw_time = time + (throw_parameter->throw_duration_share_ * throw_parameter->movement_cycle_frequence_);

	/////  Left Hand Position
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_x, begin_throw_time, throw_parameter->throw_start_left_hand_position_.x_);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_x, release_throw_time, throw_parameter->throw_release_left_hand_position_.x_, throw_parameter->throw_velocity_.x_);

	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_y, begin_throw_time, throw_parameter->throw_start_left_hand_position_.y_);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_y, release_throw_time, throw_parameter->throw_release_left_hand_position_.y_, throw_parameter->throw_velocity_.y_);

	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_z, begin_throw_time, throw_parameter->throw_start_left_hand_position_.z_);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_z, release_throw_time, throw_parameter->throw_release_left_hand_position_.z_, throw_parameter->throw_velocity_.z_);

	/////  Left Hand Axis
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_x, begin_throw_time, throw_parameter->throw_start_left_hand_axis_.x_);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_x, release_throw_time, throw_parameter->throw_release_left_hand_axis_.x_);

	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_y, begin_throw_time, throw_parameter->throw_start_left_hand_axis_.y_);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_y, release_throw_time, throw_parameter->throw_release_left_hand_axis_.y_);

	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_z, begin_throw_time, throw_parameter->throw_start_left_hand_axis_.z_);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_z, release_throw_time, throw_parameter->throw_release_left_hand_axis_.z_);

	/////  Right Hand Position
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_x, begin_throw_time, throw_parameter->throw_start_right_hand_position_.x_);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_x, release_throw_time, throw_parameter->throw_release_right_hand_position_.x_, throw_parameter->throw_velocity_.x_);

	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_y, begin_throw_time, throw_parameter->throw_start_right_hand_position_.y_);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_y, release_throw_time, throw_parameter->throw_release_right_hand_position_.y_, throw_parameter->throw_velocity_.y_);

	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_z, begin_throw_time, throw_parameter->throw_start_right_hand_position_.z_);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_z, release_throw_time, throw_parameter->throw_release_right_hand_position_.z_, throw_parameter->throw_velocity_.z_);

	/////  Right Hand Axis
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_x, begin_throw_time, throw_parameter->throw_start_right_hand_axis_.x_);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_x, release_throw_time, throw_parameter->throw_release_right_hand_axis_.x_);

	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_y, begin_throw_time, throw_parameter->throw_start_right_hand_axis_.y_);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_y, release_throw_time, throw_parameter->throw_release_right_hand_axis_.y_);

	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_z, begin_throw_time, throw_parameter->throw_start_right_hand_axis_.z_);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_z, release_throw_time, throw_parameter->throw_release_right_hand_axis_.z_);

	/////  Trunk Position
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_x, begin_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_x, release_throw_time, 0.0);

	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_y, begin_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_y, release_throw_time, 0.0);

	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_z, begin_throw_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_z, release_throw_time, 0.0);

	/////  Trunk Axis
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_x, begin_throw_time, throw_parameter->throw_start_trunk_axis_.x_);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_x, release_throw_time, throw_parameter->throw_release_trunk_axis_.x_);

	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_y, begin_throw_time, throw_parameter->throw_start_trunk_axis_.y_); // pitch
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_y, release_throw_time, throw_parameter->throw_release_trunk_axis_.y_); // pitch

	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_z, begin_throw_time, throw_parameter->throw_start_trunk_axis_.z_); // orientation
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_z, release_throw_time, throw_parameter->throw_release_trunk_axis_.z_); // orientation

	/////  Clean Up
	time = release_throw_time;
}

void ThrowCurve::calculate_throw_conclusion_movement(double & time, std::shared_ptr<ThrowParameter> & throw_parameter)
{
    //TODO: rework of parameter
	//TODO: testing
	//TODO: cleanup

	/////  Preparation
	//Set up the trajectories for the half cycle (single step)
	double finish_time = time + ((1 - throw_parameter->pick_up_duration_share_ - throw_parameter->throw_duration_share_) * throw_parameter->movement_cycle_frequence_);

	/////  Left Hand Position
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_x, finish_time, throw_parameter->end_left_hand_position_.x_);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_y, finish_time, throw_parameter->end_left_hand_position_.y_);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_position_z, finish_time, throw_parameter->end_left_hand_position_.z_);

	/////  Left Hand Axis
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_x, finish_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_y, finish_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::left_hand_axis_z, finish_time, 0.0);

	/////  Right Hand Position
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_x, finish_time, throw_parameter->end_right_hand_position_.x_);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_y, finish_time, throw_parameter->end_right_hand_position_.y_);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_position_z, finish_time, throw_parameter->end_right_hand_position_.z_);

	/////  Right Hand Axis
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_x, finish_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_y, finish_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::right_hand_axis_z, finish_time, 0.0);

	/////  Trunk Position
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_x, finish_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_y, finish_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_position_z, finish_time, 0.0);

	/////  Trunk Axis
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_x, finish_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_y, finish_time, 0.0);
	add_point_to_spline(bitbots_splines::CurvePurpose::trunk_axis_z, finish_time, 0.0);

	/////  Clean Up
	time = finish_time;
}

void ThrowCurve::add_point_to_spline(bitbots_splines::CurvePurpose spline_purpose, double time, double position, double velocity, double acceleration)
{
	//TODO: testing
	//TODO: cleanup

	sp_spline_container_->get(spline_purpose)->add_point(time, position, velocity, acceleration);
}
