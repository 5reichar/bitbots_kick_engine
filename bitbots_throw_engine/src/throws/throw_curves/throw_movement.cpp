#include "throws/throw_curves/throw_movement.h"

#include <utility>

namespace bitbots_throw{
    ThrowMovement::ThrowMovement(std::shared_ptr<ThrowMaterial> material){
        sp_material_ = std::move(material);
        trajectory_time_ = 0.0;
    }

    void ThrowMovement::init(std::shared_ptr<ThrowService> service){
        sp_service_ = std::move(service);
    }

    std::shared_ptr<ThrowMaterial> ThrowMovement::create_material(){
        init_material();
        return sp_material_;
    }

    double ThrowMovement::init_material(){
        trajectory_time_ = 0.0;

        add_pick_up_ball_movement();
        add_throw_preparation_movement();
        add_throw_movement();
        add_throw_conclusion_movement();

        return trajectory_time_;
    }

    void ThrowMovement::add_pick_up_ball_movement(){
        /////  Preparation
        //Set up the trajectories for the half cycle (single step)
        auto movement_time = sp_service_->get_movement_time_pick_up_ball();
        double squat_time = trajectory_time_ + (movement_time / 2);
        double pick_up_ball_time = trajectory_time_ + movement_time;

        ////  Movement
        sp_material_->add_point_to_left_hand(trajectory_time_, sp_service_->get_left_arm_start());
        sp_material_->add_point_to_left_hand(squat_time, sp_service_->get_left_arm_start());
        sp_material_->add_point_to_left_hand(pick_up_ball_time, sp_service_->get_left_arm_pick_up());

        sp_material_->add_point_to_right_hand(trajectory_time_, sp_service_->get_right_arm_start());
        sp_material_->add_point_to_right_hand(squat_time, sp_service_->get_right_arm_start());
        sp_material_->add_point_to_right_hand(pick_up_ball_time, sp_service_->get_right_arm_pick_up());

        sp_material_->add_point_to_left_foot(trajectory_time_, sp_service_->get_left_foot_start());
        sp_material_->add_point_to_left_foot(squat_time, sp_service_->get_left_foot_squat());
        sp_material_->add_point_to_left_foot(pick_up_ball_time, sp_service_->get_left_foot_squat());

        sp_material_->add_point_to_right_foot(trajectory_time_, sp_service_->get_right_foot_start());
        sp_material_->add_point_to_right_foot(squat_time, sp_service_->get_right_foot_squat());
        sp_material_->add_point_to_right_foot(pick_up_ball_time, sp_service_->get_right_foot_squat());
        
        /////  Clean Up
        trajectory_time_ = pick_up_ball_time;
        sp_material_->add_movement_stage(trajectory_time_);
    }

    void ThrowMovement::add_throw_preparation_movement(){
        /////  Preparation
        //Set up the trajectories for the half cycle (single step)
        auto movement_time = sp_service_->get_movement_time_throw_preparation();
        auto move_ball_at_head_height = trajectory_time_ + (movement_time / 3);
        auto move_ball_over_the_height = trajectory_time_ + (2 * movement_time / 3);
        double begin_throw_time = trajectory_time_ + movement_time;

        ////  Movement
        sp_material_->add_point_to_left_hand(move_ball_at_head_height, sp_service_->get_left_arm_ball_at_head_height());
        sp_material_->add_point_to_left_hand(move_ball_over_the_height, sp_service_->get_left_arm_throw_zenith());
        sp_material_->add_point_to_left_hand(begin_throw_time, sp_service_->get_left_arm_throw_start());

        sp_material_->add_point_to_right_hand(move_ball_at_head_height, sp_service_->get_right_arm_ball_at_head_height());
        sp_material_->add_point_to_right_hand(move_ball_over_the_height, sp_service_->get_right_arm_throw_zenith());
        sp_material_->add_point_to_right_hand(begin_throw_time, sp_service_->get_right_arm_throw_start());

        sp_material_->add_point_to_left_foot(begin_throw_time, sp_service_->get_left_foot_start());
        sp_material_->add_point_to_right_foot(begin_throw_time, sp_service_->get_right_foot_start());

        /////  Clean Up
        trajectory_time_ = begin_throw_time;
        sp_material_->add_movement_stage(trajectory_time_);
    }

    void ThrowMovement::add_throw_movement(){
        /////  Preparation
        //Set up the trajectories for the half cycle (single step)
        auto movement_time = sp_service_->get_movement_time_throw();
        double zenith_throw_time = trajectory_time_ + (movement_time / 2);
        double release_throw_time = trajectory_time_ + movement_time;

        ////  Movement
        sp_material_->add_point_to_left_hand(zenith_throw_time, sp_service_->get_left_arm_throw_zenith());
        sp_material_->add_point_to_left_hand(release_throw_time, sp_service_->get_left_arm_throw_release());
        sp_material_->add_point_to_right_hand(zenith_throw_time, sp_service_->get_right_arm_throw_zenith());
        sp_material_->add_point_to_right_hand(release_throw_time, sp_service_->get_right_arm_throw_release());

        /////  Clean Up
        trajectory_time_ = release_throw_time;
        sp_material_->add_movement_stage(trajectory_time_);
    }

    void ThrowMovement::add_throw_conclusion_movement(){
        /////  Preparation
        //Set up the trajectories for the half cycle (single step)
        auto movement_time = sp_service_->get_movement_time_throw_conclusion();
        double finish_time = trajectory_time_ + movement_time;

        ////  Movement
        sp_material_->add_point_to_left_hand(finish_time, sp_service_->get_left_arm_start());
        sp_material_->add_point_to_right_hand(finish_time, sp_service_->get_right_arm_start());

        /////  Clean Up
        trajectory_time_ = finish_time;
        sp_material_->add_movement_stage(trajectory_time_);
    }
} //bitbots_throw