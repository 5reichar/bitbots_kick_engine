#ifndef THROW_ENGINE_PARAMETER_H
#define THROW_ENGINE_PARAMETER_H

#include "parameter/struct3d.h"

struct ThrowEngineParameter
{
	// Full movement cycle frequency
	// (in Hz, > 0)
	double frequency_;
	// Lateral distance between the feet center
	// (in m, >= 0)
	double hand_distance_;
	// the radius of the ball
	double ball_radius_;
    // value used as gravity for calculations
    double gravity_;
	// The maximal distance the robot can throw (in m, >= 0)
	double max_throw_distance_;
	// the position the robot shall move the ball after picking it up,
    // and before starting to throw it
	struct3d throw_start_position_;
    // the positon the robot release the ball to throw it
	struct3d throw_release_position_;
	// The share of the movement cycle dedicated to picking up the ball
	double pick_up_duration_share_;
	// The share of the movement cycle dedicated to prepare the throwing the ball
	double throw_preparation_duration_share_;
	// The share of the movement cycle dedicated to throwing the ball
	double throw_duration_share_;
};

#endif