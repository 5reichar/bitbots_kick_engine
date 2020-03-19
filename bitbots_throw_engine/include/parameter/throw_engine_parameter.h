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

	// the position the robot shall move the ball after picking it up,
    // and before starting to throw it
	struct3d throw_start_position;

    // the positon the robot release the ball to throw it
	struct3d throw_release_position;
};

#endif