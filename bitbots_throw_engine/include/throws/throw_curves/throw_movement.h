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
        void reset_debug_data(bool const & debug_active);

        virtual void add_movement();
        void add_movement_starting_position();
        void add_movement_orient_to_ball();
        void add_movement_squat();
        void add_movement_reach_to_ball();
        void add_movement_pick_ball();
        void add_movement_stand_up();
        void add_movement_prepare_throw();
        void add_movement_throw();
        void add_movement_return_to_starting_position();

        void add_to_left_hand(Struct3dRPY const & position, Struct3dRPY const & velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, Struct3dRPY const & acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        void add_to_right_hand(Struct3dRPY const & position, Struct3dRPY const & velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, Struct3dRPY const & acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        void add_to_left_foot(Struct3dRPY const & position, Struct3dRPY const & velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, Struct3dRPY const & acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        void add_to_right_foot(Struct3dRPY const & position, Struct3dRPY const & velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, Struct3dRPY const & acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        void add_point_to_debug(std::vector<std::vector<bitbots_splines::Curve::Point>> & debug_points, double const & time, Struct3dRPY const & position, Struct3dRPY const & velocity, Struct3dRPY const & acceleration);

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