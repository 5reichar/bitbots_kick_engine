#ifndef BITBOTS_THROW_THROW_MOVEMENT_H
#define BITBOTS_THROW_THROW_MOVEMENT_H

#include "throw_movement_base.h"
#include <memory>
#include "throw_service.h"
#include "throw_material.h"

namespace bitbots_throw{
	class ThrowMovement : public ThrowMovementBase{
	public:
		ThrowMovement(std::shared_ptr<ThrowMaterial> material);

	protected:

        void add_movement() override;
        void add_movement_starting_position();
        void add_movement_orient_to_ball();
        void add_movement_squat();
        void add_movement_reach_to_ball();
        void add_movement_pick_ball();
        void add_movement_stand_up();
        void add_movement_prepare_throw();
        void add_movement_throw();
        void add_movement_return_to_starting_position();
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_MOVEMENT_H