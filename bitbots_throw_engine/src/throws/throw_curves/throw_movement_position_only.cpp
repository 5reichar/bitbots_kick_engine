#include "throws/throw_curves/throw_movement_position_only.h"

namespace bitbots_throw{

    ThrowMovementPositionOnly::ThrowMovementPositionOnly(std::shared_ptr<ThrowMaterial> material)
            : ThrowMovement(material){
    }

    void ThrowMovementPositionOnly::add_movement_prepare_throw(){
        throw_start_time_ = trajectory_time_;
        ThrowMovement::add_movement_prepare_throw();
    }

    void ThrowMovementPositionOnly::add_movement_throw(){
        // get Values
        auto current_throw_release_time = trajectory_time_;
        double duration_throw = sp_service_->get_movement_time_throw();
        duration_throw -= sp_service_->get_movement_offset_move_arms_away_from_ball();
        Struct3dRPY velocity = sp_service_->calculate_throw_velocity(duration_throw);

        auto left_arm_throw_release = sp_service_->get_left_arm_throw_release();
        auto diff = left_arm_throw_release.x_ - sp_service_->get_left_arm_ball_behind_head().x_;
        trajectory_time_ = throw_start_time_ + (diff / velocity.x_);

        ////  Movement
        ////==== Release ball
        add_to_left_hand( left_arm_throw_release);
        add_to_right_hand(sp_service_->get_right_arm_throw_release());

        ////==== Move arms away from the ball
        trajectory_time_ += sp_service_->get_movement_offset_move_arms_away_from_ball();
        add_to_left_hand(sp_service_->get_left_arm_move_away_from_ball(velocity.x_));
        add_to_right_hand(sp_service_->get_right_arm_move_away_from_ball(velocity.x_));

        if(current_throw_release_time > trajectory_time_){
            trajectory_time_ = current_throw_release_time;
        }
    }
} //bitbots_throw