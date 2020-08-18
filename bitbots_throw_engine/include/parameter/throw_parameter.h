#ifndef BITBOTS_THROW_THROW_PARAMETER_H
#define BITBOTS_THROW_THROW_PARAMETER_H

#include "parameter/struct3d.h"

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
		// (in s, > 0)
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
                      ,double movement_duration
                      ,double movement_share_pick_up
                      ,double movement_share_preparation
                      ,double movement_share_throw
                      ,double movement_share_conclusion
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
                      ,movement_duration_{movement_duration}
                      ,movement_share_pick_up_{movement_share_pick_up}
                      ,movement_share_preparation_{movement_share_preparation}
                      ,movement_share_throw_{movement_share_throw}
                      ,movement_share_conclusion_{movement_share_conclusion}{
		};
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_PARAMETER_H