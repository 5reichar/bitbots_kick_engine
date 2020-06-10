#ifndef BITBOTS_THROW_THROW_PARAMETER_H
#define BITBOTS_THROW_THROW_PARAMETER_H

#include "parameter/struct3d.h"

namespace bitbots_throw{
	struct ThrowParameter{
		//////		Parameter for the start of the movement

		// current position of the left hand
		Struct3d start_left_arm_position_;
		// current position of the right hand
		Struct3d start_right_arm_position_;


		//////		Parameter for the pick up movement

		// the position the left hand should go to after the the throw
		Struct3d pick_up_left_arm_position_;
		// the position the right hand should go to after the the throw
		Struct3d pick_up_right_arm_position_;
		// the rotation which the left hand should be when the ball will be picked up
		Struct3dRPY pick_up_left_arm_axis_;
		// the rotation which the right hand should be when the ball will be picked up
		Struct3dRPY pick_up_right_arm_axis_;
		// the rotation which the trunk should be when the ball will be picked up
		Struct3dRPY pick_up_trunk_axis_;


		//////		Parameter for the start of the throw

		// the position the robots left hand shall move the ball after picking it up,
		// and before starting to throw it
		Struct3d throw_start_left_arm_position_;
		// the position the robots right hand shall move the ball after picking it up,
		// and before starting to throw it
		Struct3d throw_start_right_arm_position_;
		// the rotation which the left hand should be when the throw is been started
		Struct3dRPY throw_start_left_arm_axis_;
		// the rotation which the right hand should be when the throw is been started
		Struct3dRPY throw_start_right_arm_axis_;
		// the rotation which the trunk should be when the ball will be thrown
		Struct3dRPY throw_start_trunk_axis_;


		//////		Parameter for the release of the ball in the throw movement

		// the position the robots left hand release the ball to throw it
		Struct3d throw_release_left_arm_position_;
		// the position the robots right hand release the ball to throw it
		Struct3d throw_release_right_arm_position_;
		// the rotation which the left hand should be when the ball will released in the throw movement
		Struct3dRPY throw_release_left_arm_axis_;
		// the rotation which the right hand should be when the ball will released in the throw movement
		Struct3dRPY throw_release_right_arm_axis_;
		// the rotation which the trunk should be when the ball will be thrown
		Struct3dRPY throw_release_trunk_axis_;
		// the velocity the ball should have when the robot throw it
		Struct3d throw_velocity_;


		//////		Parameter for the end of the movement

		// the position the left hand should go to after the the throw
		Struct3d end_left_arm_position_;
		// the position the right hand should go to after the the throw
		Struct3d end_right_arm_position_;


		//////		Additional parameter

		// position to which the ball shall be thrown
		Struct3d throw_goal_position_;


		//////		Timing parameter

		// Full movement cycle frequency
		// (in Hz, > 0)
		double movement_cycle_frequency_;
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
		ThrowParameter(Struct3d start_left_hand_position,
					Struct3d start_right_hand_position,
					Struct3d pick_up_left_hand_position,
					Struct3d pick_up_right_hand_position,
					Struct3dRPY pick_up_left_hand_axis,
					Struct3dRPY pick_up_right_hand_axis,
					Struct3dRPY pick_up_trunk_axis,
					Struct3d throw_start_left_hand_position,
					Struct3d throw_start_right_hand_position,
					Struct3dRPY throw_start_left_hand_axis,
					Struct3dRPY throw_start_right_hand_axis,
					Struct3dRPY throw_start_trunk_axis,
					Struct3d throw_release_left_hand_position,
					Struct3d throw_release_right_hand_position,
					Struct3dRPY throw_release_left_hand_axis,
					Struct3dRPY throw_release_right_hand_axis,
					Struct3dRPY throw_release_trunk_axis,
					Struct3d throw_velocity,
					Struct3d end_left_hand_position,
					Struct3d end_right_hand_position,
					Struct3d throw_goal_position,
					double movement_cycle_frequency,
					double pick_up_duration_share,
					double throw_preparation_duration_share,
					double throw_duration_share,
					double throw_conclusion_duration_share,
					double throw_angle)
					:
                start_left_arm_position_{start_left_hand_position},
                start_right_arm_position_{start_right_hand_position},
                pick_up_left_arm_position_{pick_up_left_hand_position},
                pick_up_right_arm_position_{pick_up_right_hand_position},
                pick_up_left_arm_axis_{pick_up_left_hand_axis},
                pick_up_right_arm_axis_{pick_up_right_hand_axis},
                pick_up_trunk_axis_{pick_up_trunk_axis},
                throw_start_left_arm_position_{throw_start_left_hand_position},
                throw_start_right_arm_position_{throw_start_right_hand_position},
                throw_start_left_arm_axis_{throw_start_left_hand_axis},
                throw_start_right_arm_axis_{throw_start_right_hand_axis},
                throw_start_trunk_axis_{throw_start_trunk_axis},
                throw_release_left_arm_position_{throw_release_left_hand_position},
                throw_release_right_arm_position_{throw_release_right_hand_position},
                throw_release_left_arm_axis_{throw_release_left_hand_axis},
                throw_release_right_arm_axis_{throw_release_right_hand_axis},
                throw_release_trunk_axis_{throw_release_trunk_axis},
                throw_velocity_{throw_velocity},
                end_left_arm_position_{end_left_hand_position},
                end_right_arm_position_{end_right_hand_position},
                throw_goal_position_{throw_goal_position},
                movement_cycle_frequency_{movement_cycle_frequency},
                pick_up_duration_share_{pick_up_duration_share},
                throw_preparation_duration_share_{throw_preparation_duration_share},
                throw_duration_share_{throw_duration_share},
                throw_conclusion_duration_share_{throw_conclusion_duration_share},
                throw_angle_{throw_angle}{
		}
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_PARAMETER_H