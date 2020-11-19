#ifndef BITBOTS_THROW_THROW_MOVEMENT_H
#define BITBOTS_THROW_THROW_MOVEMENT_H

#include "throw_movement_base.h"

namespace bitbots_throw{
	class ThrowMovement : public ThrowMovementBase{
	public:
		ThrowMovement(std::shared_ptr<ThrowMaterial> material);

	protected:

        void add_movement() override;
        virtual void add_pick_up_ball_movement();
        virtual void add_preparation_movement();
        virtual void add_throw_movement();
        virtual void add_conclusion_movement();

        virtual void add_movement_starting_position();
        virtual void add_movement_orient_to_ball();
        virtual void add_movement_squat();
        virtual void add_movement_squat_tilt();
        virtual void add_movement_reach_to_ball();
        virtual void add_movement_pick_ball();
        virtual void add_movement_lift_ball();
        virtual void add_movement_remove_squat_tilt();
        virtual void add_movement_stand_up();
        virtual void add_movement_orient_to_goal();
        virtual void add_movement_enter_stable_stand();
        virtual void add_movement_ball_over_head();
        virtual void add_movement_prepare_throw();
        virtual void add_movement_throw();
        virtual void add_movement_return_to_starting_position();
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_MOVEMENT_H