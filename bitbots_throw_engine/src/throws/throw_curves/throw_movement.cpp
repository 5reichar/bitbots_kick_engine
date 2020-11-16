#include "throws/throw_curves/throw_movement.h"

#include <utility>

namespace bitbots_throw{
    ThrowMovement::ThrowMovement(std::shared_ptr<ThrowMaterial> material)
        :ThrowMovementBase(material){
    }

    void ThrowMovement::add_movement(){
        add_movement_starting_position();

        auto movement_time = sp_service_->get_movement_time_pick_up_ball() / 4;
        //trajectory_time_ += movement_time;
        //add_movement_orient_to_ball();
        trajectory_time_ += movement_time;
        add_movement_squat();
        trajectory_time_ += movement_time;
        add_movement_squat_tilt();
        trajectory_time_ += movement_time;
        add_movement_reach_to_ball();
        trajectory_time_ += movement_time;
        add_movement_pick_ball();

        movement_time = sp_service_->get_movement_time_throw_preparation() / 4;
        trajectory_time_ += movement_time;
        add_movement_remove_squat_tilt();
        trajectory_time_ += movement_time;
        add_movement_stand_up();
        //trajectory_time_ += movement_time;
        //add_movement_orient_to_goal();
        trajectory_time_ += movement_time;
        add_movement_lift_ball();
        trajectory_time_ += movement_time;
        add_movement_prepare_throw();

        movement_time = sp_service_->get_movement_time_throw();
        trajectory_time_ += movement_time;
        add_movement_throw();

        movement_time = sp_service_->get_movement_time_throw_conclusion();
        trajectory_time_ += movement_time;
        add_movement_return_to_starting_position();
    }

    void ThrowMovement::add_movement_starting_position(){
        add_to_left_hand(sp_service_->get_left_arm_start());
        add_to_right_hand(sp_service_->get_right_arm_start());
        add_to_left_foot(sp_service_->get_left_foot_start());
        add_to_right_foot(sp_service_->get_right_foot_start());
    }

    void ThrowMovement::add_movement_orient_to_ball(){
        add_to_left_hand(sp_service_->get_left_arm_start());
        add_to_right_hand(sp_service_->get_right_arm_start());
        add_to_left_foot(sp_service_->get_left_foot_orientation_to_ball());
        add_to_right_foot(sp_service_->get_right_foot_orientation_to_ball());
    }

    void ThrowMovement::add_movement_squat(){
        add_to_left_hand(sp_service_->get_left_arm_start());
        add_to_right_hand(sp_service_->get_right_arm_start());
        add_to_left_foot(sp_service_->get_left_foot_squat(false));
        add_to_right_foot(sp_service_->get_right_foot_squat(false));
    }

    void ThrowMovement::add_movement_squat_tilt(){
        add_to_left_foot(sp_service_->get_left_foot_squat(true));
        add_to_right_foot(sp_service_->get_right_foot_squat(true));
    }

    void ThrowMovement::add_movement_reach_to_ball(){
        add_to_left_hand(sp_service_->get_left_arm_reach_to_ball());
        add_to_right_hand(sp_service_->get_right_arm_reach_to_ball());
    }

    void ThrowMovement::add_movement_pick_ball(){
        add_to_left_hand(sp_service_->get_left_arm_pick_up());
        add_to_right_hand(sp_service_->get_right_arm_pick_up());
    }

    void ThrowMovement::add_movement_lift_ball(){
        add_to_left_hand(sp_service_->get_left_arm_ball_at_head_height());
        add_to_right_hand(sp_service_->get_right_arm_ball_at_head_height());
    }

    void ThrowMovement::add_movement_remove_squat_tilt(){
        add_to_left_foot(sp_service_->get_left_foot_squat(false));
        add_to_right_foot(sp_service_->get_right_foot_squat(false));
    }

    void ThrowMovement::add_movement_stand_up(){
        add_to_left_foot(sp_service_->get_left_foot_start());
        add_to_right_foot(sp_service_->get_right_foot_start());
    }

    void ThrowMovement::add_movement_orient_to_goal(){
        add_to_left_foot(sp_service_->get_left_foot_orientation_to_goal());
        add_to_right_foot(sp_service_->get_right_foot_orientation_to_goal());
    }

    void ThrowMovement::add_movement_enter_stable_stand(){
        add_to_left_foot(sp_service_->get_left_foot_stable_stand());
        add_to_right_foot(sp_service_->get_right_foot_stable_stand());
    }

    void ThrowMovement::add_movement_prepare_throw(){
        add_to_left_hand(sp_service_->get_left_arm_throw_start());
        add_to_right_hand(sp_service_->get_right_arm_throw_start());
    }

    void ThrowMovement::add_movement_throw(){
        // get Values
        double duration_throw = sp_service_->get_movement_time_throw();
        duration_throw -= sp_service_->get_movement_offset_move_arms_away_from_ball();
        Struct3dRPY velocity = sp_service_->calculate_throw_velocity(duration_throw);

        ////  Movement
        ////==== Release ball
        trajectory_time_ -= sp_service_->get_movement_offset_move_arms_away_from_ball();
        add_to_left_hand( sp_service_->get_left_arm_throw_release(), velocity, velocity/duration_throw);
        add_to_right_hand(sp_service_->get_right_arm_throw_release(), velocity, velocity/duration_throw);

        ////==== Move arms away from the ball
        trajectory_time_ += sp_service_->get_movement_offset_move_arms_away_from_ball();
        add_to_left_hand(sp_service_->get_left_arm_move_away_from_ball(velocity), velocity, velocity/duration_throw);
        add_to_right_hand(sp_service_->get_right_arm_move_away_from_ball(velocity), velocity, velocity/duration_throw);
    }

    void ThrowMovement::add_movement_return_to_starting_position(){
        add_to_left_hand(sp_service_->get_left_arm_start());
        add_to_right_hand(sp_service_->get_right_arm_start());
        add_to_left_foot(sp_service_->get_left_foot_start());
        add_to_right_foot(sp_service_->get_right_foot_start());
    }
} //bitbots_throw