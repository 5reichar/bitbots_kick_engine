#ifndef BITBOTS_THROW_THROW_TYPE_PARAMETER_H
#define BITBOTS_THROW_THROW_TYPE_PARAMETER_H

#include <map>
#include <memory>

namespace bitbots_throw{
    enum ThrowTypeId : uint8_t{
        none = 0,
        throw_1,
        throw_2,
        throw_3,
        throw_4
    };
    enum ThrowMovementId : uint8_t {
        testing = 0,
        throw_movement,
        throw_movement_position_only
    };
    enum ThrowCurveId : uint8_t {
        beziercurve = 0,
        linear_spline,
        cubic_spline,
        smooth_spline
    };

	struct ThrowType{
		// Shall this kick be used
		bool active_;
		// Parameter to control if there are two or more throws that are eligable which shall be used first.
		// the rule is: the throw with the lowest level will be used
		int throw_priority_level_;
        // The movement that should used
        ThrowMovementId movement_;
        // The curves that shall be used for the arms in this movement
        ThrowCurveId arms_curve_;
        // The curves that shall be used for the legs in this movement
        ThrowCurveId legs_curve_;
        // How many percent of the max_throw_velocity shall be used if possible
        double throw_strength_;
        // The Angle at which the ball shall thrown
        double throw_angle_;
        // The minimum distance this throw should be used for
		double min_throw_distance_;
        // The maximum distance this throw should be used for
		double max_throw_distance_;
		// The amount of deviation from the goal that is tolerated
        double goal_tolerance_;
        // The amount the reference velocity will be increased/decreased to find the fitting velocity
        double velocity_adaptation_rate_;
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
		// The amount of time to move the arms away from the ball
        double movement_offset_move_arms_away_from_ball_;


		//////		Constructor
		ThrowType(bool active
		         ,int throw_priority_level
                 ,ThrowMovementId movement
                 ,ThrowCurveId arms_curve
                 ,ThrowCurveId legs_curve
                 ,double min_throw_distance
                 ,double max_throw_distance
                 ,double throw_strength
                 ,double throw_angle
                 ,double goal_tolerance
                 ,double velocity_adaptation_rate
                 ,double movement_duration
                 ,double movement_share_pick_up
                 ,double movement_share_preparation
                 ,double movement_share_throw
                 ,double movement_share_conclusion
                 ,double movement_offset_move_arms_away_from_ball
				 )
				 :active_{active}
                 ,throw_priority_level_{throw_priority_level}
                 ,movement_{movement}
                 ,arms_curve_{arms_curve}
                 ,legs_curve_{legs_curve}
                 ,throw_strength_{throw_strength}
                 ,throw_angle_{throw_angle}
                 ,min_throw_distance_{min_throw_distance}
                 ,max_throw_distance_{max_throw_distance}
                 ,goal_tolerance_{goal_tolerance}
                 ,velocity_adaptation_rate_{velocity_adaptation_rate}
                 ,movement_duration_(movement_duration)
                 ,movement_share_pick_up_{movement_share_pick_up}
                 ,movement_share_preparation_{movement_share_preparation}
                 ,movement_share_throw_{movement_share_throw}
                 ,movement_share_conclusion_{movement_share_conclusion}
                 ,movement_offset_move_arms_away_from_ball_{movement_offset_move_arms_away_from_ball}{
		}
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_TYPE_PARAMETER_H