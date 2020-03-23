#ifndef THROW_PARAMETER_H
#define THROW_PARAMETER_H

#include "parameter/struct3d.h"

struct ThrowParameter
{
	// current position of the left hand
	struct3d left_hand_start_position_;

	// the position the left hand should go to after the the throw
	struct3d left_hand_end_position_;

	// current position of the right hand
	struct3d right_hand_start_position_;

	// the position the right hand should go to after the the throw
	struct3d right_hand_end_position_;

	// current position of the ball, which the robot shall throw
	struct3d ball_position_;

	// position to which the ball shall be thrown
	struct3d throw_goal_position_;

	// the position the robot shall move the ball after picking it up,
    // and before starting to throw it
	struct3d throw_start_position_;

    // the positon the robot release the ball to throw it
	struct3d throw_release_position_;

	// the velocity the ball should have when the robot throw it
	double throw_velocity_;

	// the angle in which the upper body must be, to pick up the ball
	double pick_up_orientation_;

	// the angle in which the upper body shall bow to pick up the ball
	double pick_up_bow_angle_;

	// the angle at which the upper body must be, to make the throw
	double throw_orientation_;

	// the angle the in which the upper boddy should be when the throw movement starts 
	double throw_start_pitch_;

	// the angle the in which the upper boddy should be when the ball is release in the throw movement
	double throw_release_pitch_;

	// The share of the movement cycle dedicated to picking up the ball
	double pick_up_duration_share_;

	// The share of the movement cycle dedicated to prepare the throwing the ball
	double throw_preparation_duration_share_;

	// The share of the movement cycle dedicated to throwing the ball
	double throw_duration_share_;

	// Full movement cycle frequency
	// (in Hz, > 0)
	double movement_cycle_frequence_;
};

#endif