#ifndef BITBOTS_THROW_THROW_TYPE_PARAMETER_H
#define BITBOTS_THROW_THROW_TYPE_PARAMETER_H

#include <map>
#include <memory>

namespace bitbots_throw{
	enum ThrowTypeId : uint8_t {
	    none = 0,
		beziercurve = 1,
		linear_spline,
		cubic_spline,
		smooth_spline,
		testing
	};

	struct ThrowType{
		// Id of the type of Kick
		ThrowTypeId id_;
		// Shall this kick be used
		bool active_;
		// Parameter to control if there are two or more throws that are eligable which shall be used first.
		// the rule is: the throw with the lowest level will be used
		int throw_priority_level_;
        // How many percent of the max_throw_velocity shall be used if possible
        double throw_strength_;
        // The Angle at which the ball shall thrown
        double throw_angle_;
        // The minimum distance this throw should be used for
		double min_throw_distance_;
        // The maximum distance this throw should be used for
		double max_throw_distance_;
		// Duration of the full movement cycle (in s, > 0)
		double movement_duration_;
        // The share of the movement cycle dedicated to picking up the ball
		double movement_share_pick_up_;
        // The share of the movement cycle dedicated to prepare the throwing the ball
		double movement_share_preparation_;
        // The share of the movement cycle dedicated to throwing the ball
		double movement_share_throw_;
        // The share of the movement cycle dedicated to motion after throwing the ball
		double movement_share_conclusion_;


		//////		Constructor
		ThrowType(ThrowTypeId id
		         ,bool active
		         ,int throw_priority_level
                 ,double throw_strength
                 ,double throw_angle
                 ,double min_throw_distance
                 ,double max_throw_distance
                 ,double movement_duration
                 ,double movement_share_pick_up
                 ,double movement_share_preparation
                 ,double movement_share_throw
                 ,double movement_share_conclusion
				 )
				 :id_{id}
				 ,active_{active}
                 ,throw_priority_level_{throw_priority_level}
                 ,throw_strength_{throw_strength}
                 ,throw_angle_{throw_angle}
                 ,min_throw_distance_{min_throw_distance}
                 ,max_throw_distance_{max_throw_distance}
                 ,movement_duration_(movement_duration)
                 ,movement_share_pick_up_{movement_share_pick_up}
                 ,movement_share_preparation_{movement_share_preparation}
                 ,movement_share_throw_{movement_share_throw}
                 ,movement_share_conclusion_{movement_share_conclusion}{
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