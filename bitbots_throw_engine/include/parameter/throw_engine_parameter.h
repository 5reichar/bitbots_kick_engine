#ifndef BITBOTS_THROW_THROW_ENGINE_PARAMETER_H
#define BITBOTS_THROW_THROW_ENGINE_PARAMETER_H

#include <bitbots_throw/throw_engine_paramsConfig.h>

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
        // the safety distance between the botton of the trunk and the feet of the robot
        double squat_safety_distance_;

		//////		Constructor
		ThrowEngineParameter(bitbots_throw::throw_engine_paramsConfig& config, uint32_t level){
            gravity_ = config.gravity;
            head_height_ = config.head_height;
            trunk_height_ = config.trunk_height;
            leg_length_ = config.leg_length;
            arm_length_ = config.arm_length;
            arm_max_stall_torque_ = config.arm_max_stall_torque;
            arm_stall_torque_usage_ = config.arm_stall_torque_usage;
            ball_radius_ = config.ball_radius;
            ball_weight_ = config.ball_weight;
            squat_safety_distance_ = config.squat_safety_distance;
		}
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_ENGINE_PARAMETER_H