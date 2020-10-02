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
        std::shared_ptr<ThrowMaterial> create_material(bool debug_active = false);

        std::vector<std::vector<bitbots_splines::Curve::Point>> get_left_hand_points();
        std::vector<std::vector<bitbots_splines::Curve::Point>> get_right_hand_points();
        std::vector<std::vector<bitbots_splines::Curve::Point>> get_left_foot_points();
        std::vector<std::vector<bitbots_splines::Curve::Point>> get_right_foot_points();

	protected:
        virtual double init_material();
        void add_pick_up_ball_movement();
        void add_throw_preparation_movement();
        void add_throw_movement();
        void add_throw_conclusion_movement();

        void add_to_left_hand(double const & time, Struct3dRPY const & position, Struct3dRPY const & velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, Struct3dRPY const & acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        void add_to_right_hand(double const & time, Struct3dRPY const & position, Struct3dRPY const & velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, Struct3dRPY const & acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        void add_to_left_foot(double const & time, Struct3dRPY const & position, Struct3dRPY const & velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, Struct3dRPY const & acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        void add_to_right_foot(double const & time, Struct3dRPY const & position, Struct3dRPY const & velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, Struct3dRPY const & acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

        double trajectory_time_;
        std::shared_ptr<ThrowService> sp_service_;
        std::shared_ptr<ThrowMaterial> sp_material_;

        bool debug_active_;
        std::vector<std::vector<bitbots_splines::Curve::Point>> left_hand_points_;
        std::vector<std::vector<bitbots_splines::Curve::Point>> right_hand_points_;
        std::vector<std::vector<bitbots_splines::Curve::Point>> left_foot_points_;
        std::vector<std::vector<bitbots_splines::Curve::Point>> right_foot_points_;
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_MOVEMENT_H