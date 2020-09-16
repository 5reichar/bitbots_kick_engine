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
        std::shared_ptr<ThrowMaterial> get_material() const;

		/**
		 * Calculates the trajectory for the throw movement.
		 *
		 * @param throw_parameter Container with all values need for the calculate.
		 * @return duration of the movement. Returns 0.0 if the check for the parameter fails.
		 */
		double calculate_trajectories();

	protected:
		virtual void calculate_pick_up_ball_movement();
        virtual void calculate_throw_preparation_movement();
        virtual void calculate_throw_movement();
        virtual void calculate_throw_conclusion_movement();

        double trajectory_time_{};

        std::shared_ptr<ThrowService> sp_service_;
        std::shared_ptr<ThrowMaterial> sp_material_;
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_MOVEMENT_H