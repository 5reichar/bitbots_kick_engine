#ifndef BITBOTS_THROW_THROW_ENGINE_PARAMETER_H
#define BITBOTS_THROW_THROW_ENGINE_PARAMETER_H

#include <utility>

#include "parameter/struct3d.h"
#include "parameter/throw_type_parameter.h"

namespace bitbots_throw{
	struct ThrowEngineParameter{

        // value used as gravity for calculations
        double gravity_;
        // The height of the robots head, measure from the shoulders
        double head_height_;
        // The height of the robots trunk
        double trunk_height_;
        // The lenght of the legs of the robot
        double leg_length_;
        // The length of the arms of the robot
        double arm_length_;
        // The maximal stall torque the robot arm motor has (in Nm, >= 0)
        double arm_max_stall_torque_;
        // The recommended fraction of the stall torque the robot arm motor should use
        double arm_stall_torque_usage_;
        // the radius of the ball
        double ball_radius_;
        // the weight of the ball
        double ball_weight_;

        // The Values for the default throw
        std::shared_ptr<ThrowType> default_throw_;

		//////		Constructor
		ThrowEngineParameter(double gravity
                            ,double head_height
                            ,double trunk_height
                            ,double leg_length
                            ,double arm_length
                            ,double arm_max_stall_torque
                            ,double arm_stall_torque_usage
                            ,double ball_radius
                            ,double ball_weight
                            ,std::shared_ptr<ThrowType> default_throw
							)
							:gravity_(gravity)
                            ,head_height_(head_height)
                            ,trunk_height_(trunk_height)
                            ,leg_length_(leg_length)
							,arm_length_(arm_length)
							,arm_max_stall_torque_(arm_max_stall_torque)
							,arm_stall_torque_usage_(arm_stall_torque_usage)
							,ball_radius_(ball_radius)
							,ball_weight_(ball_weight)
                            ,default_throw_(std::move(default_throw)){
		}
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_ENGINE_PARAMETER_H