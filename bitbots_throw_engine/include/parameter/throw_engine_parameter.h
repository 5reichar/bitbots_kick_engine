#ifndef BITBOTS_THROW_THROW_ENGINE_PARAMETER_H
#define BITBOTS_THROW_THROW_ENGINE_PARAMETER_H

#include "parameter/struct3d.h"

namespace bitbots_throw{
	struct ThrowEngineParameter{
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
		Struct3d throw_start_position_;
		// the positon the robot release the ball to throw it
		Struct3d throw_release_position_;
		// The share of the movement cycle dedicated to picking up the ball
		double pick_up_duration_share_;
		// The share of the movement cycle dedicated to prepare the throwing the ball
		double throw_preparation_duration_share_;
		// The share of the movement cycle dedicated to throwing the ball
		double throw_duration_share_;
		// The share of the movement cycle dedicated to motion after throwing the ball
		double throw_conclusion_duration_share_;
		// The Angle at which the ball shall thrown
		double throw_anlge_;

		//////		Constructor
		ThrowEngineParameter(double frequency,
							double hand_distance,
							double ball_radius,
							double gravity,
							double max_throw_distance,
							Struct3d throw_start_position,
							Struct3d throw_release_position,
							double pick_up_duration_share,
							double throw_preparation_duration_share,
							double throw_duration_share,
							double throw_conclusion_duration_share,
							double throw_anlge)
							:
							frequency_{frequency},
							hand_distance_{hand_distance},
							ball_radius_{ball_radius},
							gravity_{gravity},
							max_throw_distance_{max_throw_distance},
							throw_start_position_{throw_start_position},
							throw_release_position_{throw_release_position},
							pick_up_duration_share_{pick_up_duration_share},
							throw_preparation_duration_share_{throw_preparation_duration_share},
							throw_duration_share_{throw_duration_share},
							throw_conclusion_duration_share_{throw_conclusion_duration_share},
							throw_anlge_{throw_anlge}{
		}
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_ENGINE_PARAMETER_H