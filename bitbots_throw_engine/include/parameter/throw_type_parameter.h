#ifndef BITBOTS_THROW_THROW_TYPE_PARAMETER_H
#define BITBOTS_THROW_THROW_TYPE_PARAMETER_H

#include <map>
#include <memory>

namespace bitbots_throw{
	enum ThrowTypeId{
		beziercurve = 1,
		linear_spline,
		cubic_spline,
		smooth_spline
	};

	struct ThrowType{
		// Id of the type of Kick
		ThrowTypeId id_;
		// Shall this kick be used
		bool active_;
		// Parameter to control if there are two or more throws that are eligable which shall be used first.
		// the rule is: the throw with the lowest level will be used
		int throw_priority_level_;
		// The minimum distance this throw should be used for
		double min_throw_distance_;
		// The maximum distance this throw should be used for
		double max_throw_distance_;
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
		ThrowType(ThrowTypeId id,
				bool active,
				int throw_priority_level,
				double min_throw_distance,
				double max_throw_distance,
				double pick_up_duration_share,
				double throw_preparation_duration_share,
				double throw_duration_share,
				double throw_conclusion_duration_share,
				double throw_anlge)
				:
				id_{id},
				active_{active},
				throw_priority_level_{throw_priority_level},
				min_throw_distance_{min_throw_distance},
				max_throw_distance_{max_throw_distance},
				pick_up_duration_share_{pick_up_duration_share},
				throw_preparation_duration_share_{throw_preparation_duration_share},
				throw_duration_share_{throw_duration_share},
				throw_conclusion_duration_share_{throw_conclusion_duration_share},
				throw_anlge_{throw_anlge}{
		}
	};

	struct ThrowTypeParameter
	{
		ThrowTypeId default_throw_id_;
		std::map<ThrowTypeId, std::shared_ptr<ThrowType>> map_throw_types_;

		ThrowTypeParameter(ThrowTypeId default_throw_id)
						:
						default_throw_id_{default_throw_id}{
		}
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_TYPE_PARAMETER_H