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
        // The height of the robot
        double robot_height_;
        // The radius that objects should enter to prevent collisions with the head of the robot
        double head_collision_security_radius_;
        // The length of the arms of the robot
        double arm_length_;
		// The share of the movement cycle dedicated to picking up the ball
		double pick_up_duration_share_;
		// The share of the movement cycle dedicated to prepare the throwing the ball
		double throw_preparation_duration_share_;
		// The share of the movement cycle dedicated to throwing the ball
		double throw_duration_share_;
		// The share of the movement cycle dedicated to motion after throwing the ball
		double throw_conclusion_duration_share_;
		// The Angle at which the ball shall thrown
		double throw_angle_;

		//////		Constructor
		ThrowEngineParameter(double frequency
		                    ,double hand_distance
		                    ,double ball_radius
		                    ,double gravity
							,double max_throw_distance
							,double robot_height
							,double head_collision_security_radius
                            ,double arm_length
                            ,double pick_up_duration_share
							,double throw_preparation_duration_share
							,double throw_duration_share
							,double throw_conclusion_duration_share
							,double throw_angle
							)
							:frequency_{frequency}
							,hand_distance_{hand_distance}
							,ball_radius_{ball_radius}
							,gravity_{gravity}
							,max_throw_distance_{max_throw_distance}
							,robot_height_{robot_height}
                            ,head_collision_security_radius_{head_collision_security_radius}
                            ,arm_length_{arm_length}
                            ,pick_up_duration_share_{pick_up_duration_share}
							,throw_preparation_duration_share_{throw_preparation_duration_share}
							,throw_duration_share_{throw_duration_share}
							,throw_conclusion_duration_share_{throw_conclusion_duration_share}
							,throw_angle_{throw_angle}{
		}
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_ENGINE_PARAMETER_H