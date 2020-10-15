#ifndef BITBOTS_THROW_THROW_SERVICE_H
#define BITBOTS_THROW_THROW_SERVICE_H

#include "utility/throw_utilities.h"
#include "parameter/throw_type_parameter.h"
#include "parameter/throw_engine_parameter.h"

namespace bitbots_throw{
    class ThrowService{
    public:
        ThrowService(ThrowRequest request, ThrowType throw_type, RobotAndWorldParameter robot_and_world_parameter);

        virtual Struct3dRPY get_left_arm_start();
        virtual Struct3dRPY get_left_arm_reach_to_ball();
        virtual Struct3dRPY get_left_arm_pick_up();
        virtual Struct3dRPY get_left_arm_ball_at_head_height();
        virtual Struct3dRPY get_left_arm_throw_start();
        virtual Struct3dRPY get_left_arm_throw_release();
        virtual Struct3dRPY get_left_arm_move_away_from_ball();

        virtual Struct3dRPY get_right_arm_start();
        virtual Struct3dRPY get_right_arm_pick_up();
        virtual Struct3dRPY get_right_arm_reach_to_ball();
        virtual Struct3dRPY get_right_arm_ball_at_head_height();
        virtual Struct3dRPY get_right_arm_throw_start();
        virtual Struct3dRPY get_right_arm_throw_release();
        virtual Struct3dRPY get_right_arm_move_away_from_ball();

        virtual Struct3dRPY get_left_foot_start();
        virtual Struct3dRPY get_left_foot_orientation_to_ball();
        virtual Struct3dRPY get_left_foot_squat();
        virtual Struct3dRPY get_left_foot_orientation_to_goal();

        virtual Struct3dRPY get_right_foot_start();
        virtual Struct3dRPY get_right_foot_orientation_to_ball();
        virtual Struct3dRPY get_right_foot_squat();
        virtual Struct3dRPY get_right_foot_orientation_to_goal();

        virtual double get_movement_time_pick_up_ball();
        virtual double get_movement_time_throw_preparation();
        virtual double get_movement_time_throw();
        virtual double get_movement_time_throw_conclusion();
        virtual double get_movement_offset_move_arms_away_from_ball();

        virtual Struct3dRPY calculate_throw_velocity(double const & duration_build_up_velocity);

    protected:
        double calculate_robot_max_velocity(double const & duration_build_up_velocity) const;

        virtual Struct3dRPY create_reach_to_ball();
        virtual Struct3dRPY create_pick_up();
        virtual Struct3dRPY create_ball_at_head_height();
        virtual Struct3dRPY create_throw_start();
        virtual Struct3dRPY create_throw_release();
        virtual Struct3dRPY create_orientation_to_ball(bool const & left_foot, bool const & rotate_coordinate);
        virtual Struct3dRPY create_orientation_to_goal(bool const & left_foot);

        double get_lowest_foot_position();

        ThrowRequest request_;
        ThrowType throw_type_;
        RobotAndWorldParameter robot_and_world_parameter_;
    };
}
#endif //BITBOTS_THROW_THROW_SERVICE_H
