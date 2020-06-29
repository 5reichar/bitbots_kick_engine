#ifndef BITBOTS_THROW_THROW_PARAMETER_H
#define BITBOTS_THROW_THROW_PARAMETER_H

#include "parameter/struct3d.h"
#include "../../../../../../Documents/git/bitbots_kick_engine/bitbots_throw_engine/include/parameter/struct3d.h"

namespace bitbots_throw{
	struct ThrowParameter{
		//////		Parameter for the start of the movement
        // current position of the left hand
        Struct3dRPY start_left_arm_;
        // current position of the right hand
        Struct3dRPY start_right_arm_;
        // current position of the left hand
        Struct3dRPY start_left_feet_;
        // current position of the right hand
        Struct3dRPY start_right_feet_;


        //////		Parameter for the pick up movement
		// the position and rotation the left hand should go to after the the throw
		Struct3dRPY pick_up_left_arm_;
		// the position and rotation the right hand should go to after the the throw
		Struct3dRPY pick_up_right_arm_;
		// the position and rotation which the trunk should be when the ball will be picked up
		Struct3dRPY pick_up_trunk_;


		//////		Parameter for the start of the throw
		// the position and rotation the robots left hand shall move the ball after picking it up,
		// and before starting to throw it
		Struct3dRPY throw_start_left_arm_;
		// the position and rotation the robots right hand shall move the ball after picking it up,
		// and before starting to throw it
		Struct3dRPY throw_start_right_arm_;
		// the rotation which the trunk should be when the ball will be thrown
		Struct3dRPY throw_start_trunk_;


        //////		Parameter for the release of the ball in the throw movement
        // the position and rotation the robots left hand release the ball to throw it
        Struct3dRPY throw_zenith_left_arm_;
        // the position and rotation the robots right hand release the ball to throw it
        Struct3dRPY throw_zenith_right_arm_;


		//////		Parameter for the release of the ball in the throw movement
		// the position and rotation the robots left hand release the ball to throw it
		Struct3dRPY throw_release_left_arm_;
		// the position and rotation the robots right hand release the ball to throw it
		Struct3dRPY throw_release_right_arm_;
		// the position and rotation which the trunk should be when the ball will be thrown
		Struct3dRPY throw_release_trunk_;
		// the velocity the ball should have when the robot throw it
		Struct3d throw_velocity_;


		//////		Parameter for the end of the movement


		//////		Additional parameter


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


		//////		Constructor
		ThrowParameter(Struct3dRPY start_left_hand
		              ,Struct3dRPY start_right_hand
                      ,Struct3dRPY start_left_feet
                      ,Struct3dRPY start_right_feet
                      ,Struct3dRPY pick_up_left_hand
                      ,Struct3dRPY pick_up_right_hand
                      ,Struct3dRPY pick_up_trunk
                      ,Struct3dRPY throw_start_left_hand
                      ,Struct3dRPY throw_start_right_hand
                      ,Struct3dRPY throw_start_trunk
                      ,Struct3dRPY throw_zenith_left_arm
                      ,Struct3dRPY throw_zenith_right_arm
                      ,Struct3dRPY throw_release_left_hand
                      ,Struct3dRPY throw_release_right_hand
                      ,Struct3dRPY throw_release_trunk
                      ,Struct3d throw_velocity
                      ,double movement_cycle_frequency
                      ,double pick_up_duration_share
                      ,double throw_preparation_duration_share
                      ,double throw_duration_share
                      ,double throw_conclusion_duration_share
                      )
                      :start_left_arm_{start_left_hand}
                      ,start_right_arm_{start_right_hand}
                      ,start_left_feet_(start_left_feet)
                      ,start_right_feet_(start_right_feet)
                      ,pick_up_left_arm_{pick_up_left_hand}
                      ,pick_up_right_arm_{pick_up_right_hand}
                      ,pick_up_trunk_{pick_up_trunk}
                      ,throw_start_left_arm_{throw_start_left_hand}
                      ,throw_start_right_arm_{throw_start_right_hand}
                      ,throw_start_trunk_{throw_start_trunk}
                      ,throw_zenith_left_arm_{throw_zenith_left_arm}
                      ,throw_zenith_right_arm_{throw_zenith_right_arm}
                      ,throw_release_left_arm_{throw_release_left_hand}
                      ,throw_release_right_arm_{throw_release_right_hand}
                      ,throw_release_trunk_{throw_release_trunk}
                      ,throw_velocity_{throw_velocity}
                      ,movement_cycle_frequency_{movement_cycle_frequency}
                      ,pick_up_duration_share_{pick_up_duration_share}
                      ,throw_preparation_duration_share_{throw_preparation_duration_share}
                      ,throw_duration_share_{throw_duration_share}
                      ,throw_conclusion_duration_share_{throw_conclusion_duration_share}{
		};
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_PARAMETER_H