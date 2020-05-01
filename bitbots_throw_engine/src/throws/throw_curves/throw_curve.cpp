#include "throws/throw_curves/throw_curve.h"

ThrowCurve::ThrowCurve(std::shared_ptr<bitbots_splines::PoseHandle> left_hand, std::shared_ptr<bitbots_splines::PoseHandle> right_hand, std::shared_ptr<bitbots_splines::PoseHandle> trunk)
		: sp_pose_left_hand_(std::move(left_hand)),
		  sp_pose_right_hand_(std::move(right_hand)),
		  sp_pose_trunk_(std::move(trunk))
{

}

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


std::shared_ptr<bitbots_splines::PoseHandle> ThrowCurve::get_pose_left_hand() const
{
	return std::move(sp_pose_left_hand_);
}

std::shared_ptr<bitbots_splines::PoseHandle> ThrowCurve::get_pose_right_hand() const
{
	return std::move(sp_pose_right_hand_);
}

std::shared_ptr<bitbots_splines::PoseHandle> ThrowCurve::get_pose_trunk() const
{
	return std::move(sp_pose_trunk_);
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

	/////  Left Hand
	sp_pose_left_hand_->x()->add_point(start_time, throw_parameter->start_left_hand_position_.x_);
	sp_pose_left_hand_->x()->add_point(pick_up_ball_time, throw_parameter->pick_up_left_hand_position_.x_);

	sp_pose_left_hand_->y()->add_point(start_time, throw_parameter->start_left_hand_position_.y_);
	sp_pose_left_hand_->y()->add_point(pick_up_ball_time, throw_parameter->pick_up_left_hand_position_.y_);

	sp_pose_left_hand_->z()->add_point(start_time, throw_parameter->start_left_hand_position_.z_);
	sp_pose_left_hand_->z()->add_point(pick_up_ball_time, throw_parameter->pick_up_left_hand_position_.z_);

	sp_pose_left_hand_->roll()->add_point(start_time, 0.0);
	sp_pose_left_hand_->roll()->add_point(pick_up_ball_time, throw_parameter->pick_up_left_hand_axis_.x_);

	sp_pose_left_hand_->pitch()->add_point(start_time, 0.0);
	sp_pose_left_hand_->pitch()->add_point(pick_up_ball_time, throw_parameter->pick_up_left_hand_axis_.y_);

	sp_pose_left_hand_->yaw()->add_point(start_time, 0.0);
	sp_pose_left_hand_->yaw()->add_point(pick_up_ball_time, throw_parameter->pick_up_left_hand_axis_.z_);

	/////  Right Hand
	sp_pose_right_hand_->x()->add_point(start_time, throw_parameter->end_right_hand_position_.x_);
	sp_pose_right_hand_->x()->add_point(pick_up_ball_time, throw_parameter->pick_up_right_hand_position_.x_);

	sp_pose_right_hand_->y()->add_point(start_time, throw_parameter->end_right_hand_position_.y_);
	sp_pose_right_hand_->y()->add_point(pick_up_ball_time, throw_parameter->pick_up_right_hand_position_.y_);

	sp_pose_right_hand_->z()->add_point(start_time, throw_parameter->end_right_hand_position_.z_);
	sp_pose_right_hand_->z()->add_point(pick_up_ball_time, throw_parameter->pick_up_right_hand_position_.z_);

	sp_pose_right_hand_->roll()->add_point(start_time, 0.0);
	sp_pose_right_hand_->roll()->add_point(pick_up_ball_time, throw_parameter->pick_up_right_hand_axis_.x_);

	sp_pose_right_hand_->pitch()->add_point(start_time, 0.0);
	sp_pose_right_hand_->pitch()->add_point(pick_up_ball_time, throw_parameter->pick_up_right_hand_axis_.y_);

	sp_pose_right_hand_->yaw()->add_point(start_time, 0.0);
	sp_pose_right_hand_->yaw()->add_point(pick_up_ball_time, throw_parameter->pick_up_right_hand_axis_.z_);

	/////  Trunk
	sp_pose_trunk_->x()->add_point(start_time, 0.0);
	sp_pose_trunk_->x()->add_point(pick_up_ball_time, 0.0);

	sp_pose_trunk_->y()->add_point(start_time, 0.0);
	sp_pose_trunk_->y()->add_point(pick_up_ball_time, 0.0);

	sp_pose_trunk_->z()->add_point(start_time, 0.0);
	sp_pose_trunk_->z()->add_point(pick_up_ball_time, 0.0);

	sp_pose_trunk_->roll()->add_point(start_time, 0.0);
	sp_pose_trunk_->roll()->add_point(pick_up_ball_time, throw_parameter->pick_up_trunk_axis_.x_);

	sp_pose_trunk_->pitch()->add_point(start_time, 0.0);
	sp_pose_trunk_->pitch()->add_point(pick_up_ball_time, throw_parameter->pick_up_trunk_axis_.y_); // Bow Angle

	sp_pose_trunk_->yaw()->add_point(start_time, 0.0);
	sp_pose_trunk_->yaw()->add_point(pick_up_ball_time, throw_parameter->pick_up_trunk_axis_.z_); // Orientation

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

	/////  Left Hand
	sp_pose_left_hand_->x()->add_point(begin_throw_time, throw_parameter->throw_start_left_hand_position_.x_);
	sp_pose_left_hand_->x()->add_point(release_throw_time, throw_parameter->throw_release_left_hand_position_.x_, throw_parameter->throw_velocity_.x_);

	sp_pose_left_hand_->y()->add_point(begin_throw_time, throw_parameter->throw_start_left_hand_position_.y_);
	sp_pose_left_hand_->y()->add_point(release_throw_time, throw_parameter->throw_release_left_hand_position_.y_, throw_parameter->throw_velocity_.y_);

	sp_pose_left_hand_->z()->add_point(begin_throw_time, throw_parameter->throw_start_left_hand_position_.z_);
	sp_pose_left_hand_->z()->add_point(release_throw_time, throw_parameter->throw_release_left_hand_position_.z_, throw_parameter->throw_velocity_.z_);

	sp_pose_left_hand_->roll()->add_point(begin_throw_time, throw_parameter->throw_start_left_hand_axis_.x_);
	sp_pose_left_hand_->roll()->add_point(release_throw_time, throw_parameter->throw_release_left_hand_axis_.x_);

	sp_pose_left_hand_->pitch()->add_point(begin_throw_time, throw_parameter->throw_start_left_hand_axis_.y_);
	sp_pose_left_hand_->pitch()->add_point(release_throw_time, throw_parameter->throw_release_left_hand_axis_.y_);

	sp_pose_left_hand_->yaw()->add_point(begin_throw_time, throw_parameter->throw_start_left_hand_axis_.z_);
	sp_pose_left_hand_->yaw()->add_point(release_throw_time, throw_parameter->throw_release_left_hand_axis_.z_);

	/////  Right Hand
	sp_pose_right_hand_->x()->add_point(begin_throw_time, throw_parameter->throw_start_right_hand_position_.x_);
	sp_pose_right_hand_->x()->add_point(release_throw_time, throw_parameter->throw_release_right_hand_position_.x_, throw_parameter->throw_velocity_.x_);

	sp_pose_right_hand_->y()->add_point(begin_throw_time, throw_parameter->throw_start_right_hand_position_.y_);
	sp_pose_right_hand_->y()->add_point(release_throw_time, throw_parameter->throw_release_right_hand_position_.y_, throw_parameter->throw_velocity_.y_);

	sp_pose_right_hand_->z()->add_point(begin_throw_time, throw_parameter->throw_start_right_hand_position_.z_);
	sp_pose_right_hand_->z()->add_point(release_throw_time, throw_parameter->throw_release_right_hand_position_.z_, throw_parameter->throw_velocity_.z_);

	sp_pose_right_hand_->roll()->add_point(begin_throw_time, throw_parameter->throw_start_right_hand_axis_.x_);
	sp_pose_right_hand_->roll()->add_point(release_throw_time, throw_parameter->throw_release_right_hand_axis_.x_);

	sp_pose_right_hand_->pitch()->add_point(begin_throw_time, throw_parameter->throw_start_right_hand_axis_.y_);
	sp_pose_right_hand_->pitch()->add_point(release_throw_time, throw_parameter->throw_release_right_hand_axis_.y_);

	sp_pose_right_hand_->yaw()->add_point(begin_throw_time, throw_parameter->throw_start_right_hand_axis_.z_);
	sp_pose_right_hand_->yaw()->add_point(release_throw_time, throw_parameter->throw_release_right_hand_axis_.z_);

	/////  Trunk
	sp_pose_trunk_->x()->add_point(begin_throw_time, 0.0);
	sp_pose_trunk_->x()->add_point(release_throw_time, 0.0);

	sp_pose_trunk_->y()->add_point(begin_throw_time, 0.0);
	sp_pose_trunk_->y()->add_point(release_throw_time, 0.0);

	sp_pose_trunk_->z()->add_point(begin_throw_time, 0.0);
	sp_pose_trunk_->z()->add_point(release_throw_time, 0.0);

	sp_pose_trunk_->roll()->add_point(begin_throw_time, throw_parameter->throw_start_trunk_axis_.x_);
	sp_pose_trunk_->roll()->add_point(release_throw_time, throw_parameter->throw_release_trunk_axis_.x_);

	sp_pose_trunk_->pitch()->add_point(begin_throw_time, throw_parameter->throw_start_trunk_axis_.y_); // pitch
	sp_pose_trunk_->pitch()->add_point(release_throw_time, throw_parameter->throw_release_trunk_axis_.y_); // pitch

	sp_pose_trunk_->yaw()->add_point(begin_throw_time, throw_parameter->throw_start_trunk_axis_.z_); // orientation
	sp_pose_trunk_->yaw()->add_point(release_throw_time, throw_parameter->throw_release_trunk_axis_.z_); // orientation

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

	/////  Left Hand
	sp_pose_left_hand_->x()->add_point(finish_time, throw_parameter->end_left_hand_position_.x_);
	sp_pose_left_hand_->y()->add_point(finish_time, throw_parameter->end_left_hand_position_.y_);
	sp_pose_left_hand_->z()->add_point(finish_time, throw_parameter->end_left_hand_position_.z_);
	sp_pose_left_hand_->roll()->add_point(finish_time, 0.0);
	sp_pose_left_hand_->pitch()->add_point(finish_time, 0.0);
	sp_pose_left_hand_->yaw()->add_point(finish_time, 0.0);

	/////  Right Hand
	sp_pose_right_hand_->x()->add_point(finish_time, throw_parameter->end_right_hand_position_.x_);
	sp_pose_right_hand_->y()->add_point(finish_time, throw_parameter->end_right_hand_position_.y_);
	sp_pose_right_hand_->z()->add_point(finish_time, throw_parameter->end_right_hand_position_.z_);
	sp_pose_right_hand_->roll()->add_point(finish_time, 0.0);
	sp_pose_right_hand_->pitch()->add_point(finish_time, 0.0);
	sp_pose_right_hand_->yaw()->add_point(finish_time, 0.0);

	/////  Trunk
	sp_pose_trunk_->x()->add_point(finish_time, 0.0);
	sp_pose_trunk_->y()->add_point(finish_time, 0.0);
	sp_pose_trunk_->z()->add_point(finish_time, 0.0);
	sp_pose_trunk_->roll()->add_point(finish_time, 0.0);
	sp_pose_trunk_->pitch()->add_point(finish_time, 0.0);
	sp_pose_trunk_->yaw()->add_point(finish_time, 0.0);

	/////  Clean Up
	time = finish_time;
}
