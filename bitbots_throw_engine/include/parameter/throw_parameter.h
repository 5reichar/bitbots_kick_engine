#ifndef THROW_PARAMETER_H
#define THROW_PARAMETER_H

#include "parameter/struct3d.h"

struct ThrowParameter
{
	struct3d left_hand_start_position;
	struct3d left_hand_end_position;
	struct3d right_hand_start_position;
	struct3d right_hand_end_position;

	// current position of the ball, which the robot shall throw
	struct3d ball_position;

	// position to which the ball shall be thrown
	struct3d throw_goal_position;

	// the position the robot shall move the ball after picking it up,
    // and before starting to throw it
	struct3d throw_start_position;

    // the positon the robot release the ball to throw it
	struct3d throw_release_position;

	// the velocity the ball should have when the robot throw it
	double throw_velocity;

	// the angle at which the upper body must be, to make the throw
	double throw_orientation;

	// the angle in which the upper body shall bow to pick up the ball
	double pick_up_bow_angle;

	// Full movement cycle frequency
	// (in Hz, > 0)
	double movement_cycle_frequence;
};

#endif