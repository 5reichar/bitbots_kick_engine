#ifndef BITBOTS_THROW_THROW_MOVEMENT_H
#define BITBOTS_THROW_THROW_MOVEMENT_H

#include <memory>
#include "throw_service.h"
#include "throw_material.h"

namespace bitbots_throw{
	class ThrowMovement{
	public:
		ThrowMovement(std::shared_ptr<ThrowMaterial> material);

		virtual void init(std::shared_ptr<ThrowService> service);
        std::shared_ptr<ThrowMaterial> create_material();

	protected:
        virtual double init_material();
        void add_pick_up_ball_movement();
        void add_throw_preparation_movement();
        void add_throw_movement();
        void add_throw_conclusion_movement();

        double trajectory_time_{};

        std::shared_ptr<ThrowService> sp_service_;
        std::shared_ptr<ThrowMaterial> sp_material_;
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_MOVEMENT_H