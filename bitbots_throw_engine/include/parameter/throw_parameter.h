#ifndef THROW_PARAMETER_H
#define THROW_PARAMETER_H

#include "parameter/struct3d.h"

struct ThrowParameter
{
	//////		Parameter for the start of the movement

	// current position of the left hand
	struct3d start_left_hand_position_;
	// current position of the right hand
	struct3d start_right_hand_position_;


	//////		Parameter for the pick up movement

	// the position the left hand should go to after the the throw
	struct3d pick_up_left_hand_position_;
	// the position the right hand should go to after the the throw
	struct3d pick_up_right_hand_position_;
	// the rotation which the left hand should be when the ball will be picked up
	struct3d pick_up_left_hand_axis_;
	// the rotation which the right hand should be when the ball will be picked up
	struct3d pick_up_right_hand_axis_;
	// the rotation which the trunk should be when the ball will be picked up
	struct3d pick_up_trunk_axis_;


	//////		Parameter for the start of the throw

	// the position the robots left hand shall move the ball after picking it up,
    // and before starting to throw it
	struct3d throw_start_left_hand_position_;
	// the position the robots right hand shall move the ball after picking it up,
    // and before starting to throw it
	struct3d throw_start_right_hand_position_;
	// the rotation which the left hand should be when the throw is been started
	struct3d throw_start_left_hand_axis_;
	// the rotation which the right hand should be when the throw is been started
	struct3d throw_start_right_hand_axis_;
	// the rotation which the trunk should be when the ball will be thrown
	struct3d throw_start_trunk_axis_;


	//////		Parameter for the release of the ball in the throw movement

    // the positon the robots left hand release the ball to throw it
	struct3d throw_release_left_hand_position_;
    // the positon the robots right hand release the ball to throw it
	struct3d throw_release_right_hand_position_;
	// the rotation which the left hand should be when the ball will released in the throw movement
	struct3d throw_release_left_hand_axis_;
	// the rotation which the right hand should be when the ball will released in the throw movement
	struct3d throw_release_right_hand_axis_;
	// the rotation which the trunk should be when the ball will be thrown
	struct3d throw_release_trunk_axis_;
	// the velocity the ball should have when the robot throw it
	struct3d throw_velocity_;


	//////		Parameter for the end of the movement

	// the position the left hand should go to after the the throw
	struct3d end_left_hand_position_;
	// the position the right hand should go to after the the throw
	struct3d end_right_hand_position_;


	//////		Additional parameter

	// position to which the ball shall be thrown
	struct3d throw_goal_position_;



	//////		Timing parameter

	// Full movement cycle frequency
	// (in Hz, > 0)
	double movement_cycle_frequence_;
	// The share of the movement cycle dedicated to picking up the ball
	double pick_up_duration_share_;
	// The share of the movement cycle dedicated to prepare the throwing the ball
	double throw_preparation_duration_share_;
	// The share of the movement cycle dedicated to throwing the ball
	double throw_duration_share_;
};

#endif