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

    std::shared_ptr<ThrowMaterial> ThrowMovement::create_material(bool debug_active){
        debug_active_ = debug_active;
        left_hand_points_.clear();
        left_hand_points_.resize(6);
        right_hand_points_.clear();
        right_hand_points_.resize(6);
        left_foot_points_.clear();
        left_foot_points_.resize(6);
        right_foot_points_.clear();
        right_foot_points_.resize(6);

        init_material();

        return sp_material_;
    }

    std::vector<std::vector<bitbots_splines::Curve::Point>> ThrowMovement::get_left_hand_points(){
        return left_hand_points_;
    }

    std::vector<std::vector<bitbots_splines::Curve::Point>> ThrowMovement::get_right_hand_points(){
        return right_hand_points_;
    }

    std::vector<std::vector<bitbots_splines::Curve::Point>> ThrowMovement::get_left_foot_points(){
        return left_foot_points_;
    }

    std::vector<std::vector<bitbots_splines::Curve::Point>> ThrowMovement::get_right_foot_points(){
        return right_foot_points_;
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
        double orientation_time = trajectory_time_ + (movement_time / 4);
        double squat_time = trajectory_time_ + (movement_time / 2);
        double reach_time = trajectory_time_ + 3 * (movement_time / 4);
        double pick_up_ball_time = trajectory_time_ + movement_time;

        ////  Movement
        ////==== Start position
        add_to_left_hand(trajectory_time_, sp_service_->get_left_arm_start());
        add_to_right_hand(trajectory_time_, sp_service_->get_right_arm_start());
        add_to_left_foot(trajectory_time_, sp_service_->get_left_foot_start());
        add_to_right_foot(trajectory_time_, sp_service_->get_right_foot_start());

        ////==== Orient to ball
        add_to_left_hand(orientation_time, sp_service_->get_left_arm_start());
        add_to_right_hand(orientation_time, sp_service_->get_right_arm_start());
        add_to_left_foot(orientation_time, sp_service_->get_left_foot_orientation_to_ball());
        add_to_right_foot(orientation_time, sp_service_->get_right_foot_orientation_to_ball());

        ////==== Squat
        add_to_left_hand(squat_time, sp_service_->get_left_arm_start());
        add_to_right_hand(squat_time, sp_service_->get_right_arm_start());
        add_to_left_foot(squat_time, sp_service_->get_left_foot_squat());
        add_to_right_foot(squat_time, sp_service_->get_right_foot_squat());

        ////==== Reach to ball
        add_to_left_hand(reach_time, sp_service_->get_left_arm_reach_to_ball());
        add_to_right_hand(reach_time, sp_service_->get_right_arm_reach_to_ball());
        add_to_left_foot(reach_time, sp_service_->get_left_foot_squat());
        add_to_right_foot(reach_time, sp_service_->get_right_foot_squat());

        ////==== Pick ball
        add_to_left_hand(pick_up_ball_time, sp_service_->get_left_arm_pick_up());
        add_to_right_hand(pick_up_ball_time, sp_service_->get_right_arm_pick_up());
        add_to_left_foot(pick_up_ball_time, sp_service_->get_left_foot_squat());
        add_to_right_foot(pick_up_ball_time, sp_service_->get_right_foot_squat());

        /////  Clean Up
        trajectory_time_ = pick_up_ball_time;
        sp_material_->add_movement_stage(trajectory_time_);
    }

    void ThrowMovement::add_throw_preparation_movement(){
        /////  Preparation
        //Set up the trajectories for the half cycle (single step)
        auto movement_time = sp_service_->get_movement_time_throw_preparation();
        auto move_ball_up = trajectory_time_ + (movement_time / 3);
        auto move_ball_over_the_head = trajectory_time_ + (2 * movement_time / 3);
        double begin_throw_time = trajectory_time_ + movement_time;

        ////  Movement
        ////==== Move ball up
        add_to_left_hand(move_ball_up, sp_service_->get_left_arm_ball_at_head_height());
        add_to_right_hand(move_ball_up, sp_service_->get_right_arm_ball_at_head_height());
        add_to_left_foot(move_ball_up, sp_service_->get_left_foot_start());
        add_to_right_foot(move_ball_up, sp_service_->get_right_foot_start());

        ////==== Move ball over the head
        add_to_left_hand(move_ball_over_the_head, sp_service_->get_left_arm_throw_zenith());
        add_to_right_hand(move_ball_over_the_head, sp_service_->get_right_arm_throw_zenith());
        add_to_left_foot(move_ball_over_the_head, sp_service_->get_left_foot_orientation_to_goal());
        add_to_right_foot(move_ball_over_the_head, sp_service_->get_right_foot_orientation_to_goal());

        ////==== Begin throw position
        add_to_left_hand(begin_throw_time, sp_service_->get_left_arm_throw_start());
        add_to_right_hand(begin_throw_time, sp_service_->get_right_arm_throw_start());

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
        ////==== Return to ball to over head position
        add_to_left_hand(zenith_throw_time, sp_service_->get_left_arm_throw_zenith_return());
        add_to_right_hand(zenith_throw_time, sp_service_->get_right_arm_throw_zenith_return());

        ////==== Release ball
        //Struct3dRPY velocity = sp_service_->get_throw_velocity();
        add_to_left_hand(release_throw_time, sp_service_->get_left_arm_throw_release());//, velocity);
        add_to_right_hand(release_throw_time, sp_service_->get_right_arm_throw_release());//, velocity);

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
        add_to_left_hand(finish_time, sp_service_->get_left_arm_start());
        add_to_right_hand(finish_time, sp_service_->get_right_arm_start());

        /////  Clean Up
        trajectory_time_ = finish_time;
        sp_material_->add_movement_stage(trajectory_time_);
    }

    void ThrowMovement::add_to_left_hand(const double & time, const Struct3dRPY & position, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        sp_material_->add_point_to_left_hand(time, position, velocity, acceleration);

        if(debug_active_){
            left_hand_points_.at(0).emplace_back(bitbots_splines::Curve::Point{time, position.x_, velocity.x_, acceleration.x_});
            left_hand_points_.at(1).emplace_back(bitbots_splines::Curve::Point{time, position.y_, velocity.x_, acceleration.y_});
            left_hand_points_.at(2).emplace_back(bitbots_splines::Curve::Point{time, position.z_, velocity.x_, acceleration.z_});
            left_hand_points_.at(3).emplace_back(bitbots_splines::Curve::Point{time, position.roll_, velocity.x_, acceleration.roll_});
            left_hand_points_.at(4).emplace_back(bitbots_splines::Curve::Point{time, position.pitch_, velocity.pitch_, acceleration.pitch_});
            left_hand_points_.at(5).emplace_back(bitbots_splines::Curve::Point{time, position.yaw_, velocity.yaw_, acceleration.yaw_});
        }
    }

    void ThrowMovement::add_to_right_hand(const double & time, const Struct3dRPY & position, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        sp_material_->add_point_to_right_hand(time, position, velocity, acceleration);

        if(debug_active_){
            right_hand_points_.at(0).emplace_back(bitbots_splines::Curve::Point{time, position.x_, velocity.x_, acceleration.x_});
            right_hand_points_.at(1).emplace_back(bitbots_splines::Curve::Point{time, position.y_, velocity.x_, acceleration.y_});
            right_hand_points_.at(2).emplace_back(bitbots_splines::Curve::Point{time, position.z_, velocity.x_, acceleration.z_});
            right_hand_points_.at(3).emplace_back(bitbots_splines::Curve::Point{time, position.roll_, velocity.x_, acceleration.roll_});
            right_hand_points_.at(4).emplace_back(bitbots_splines::Curve::Point{time, position.pitch_, velocity.pitch_, acceleration.pitch_});
            right_hand_points_.at(5).emplace_back(bitbots_splines::Curve::Point{time, position.yaw_, velocity.yaw_, acceleration.yaw_});
        }
    }

    void ThrowMovement::add_to_left_foot(const double & time, const Struct3dRPY & position, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        sp_material_->add_point_to_left_foot(time, position, velocity, acceleration);

        if(debug_active_){
            left_foot_points_.at(0).emplace_back(bitbots_splines::Curve::Point{time, position.x_, velocity.x_, acceleration.x_});
            left_foot_points_.at(1).emplace_back(bitbots_splines::Curve::Point{time, position.y_, velocity.x_, acceleration.y_});
            left_foot_points_.at(2).emplace_back(bitbots_splines::Curve::Point{time, position.z_, velocity.x_, acceleration.z_});
            left_foot_points_.at(3).emplace_back(bitbots_splines::Curve::Point{time, position.roll_, velocity.x_, acceleration.roll_});
            left_foot_points_.at(4).emplace_back(bitbots_splines::Curve::Point{time, position.pitch_, velocity.pitch_, acceleration.pitch_});
            left_foot_points_.at(5).emplace_back(bitbots_splines::Curve::Point{time, position.yaw_, velocity.yaw_, acceleration.yaw_});
        }
    }

    void ThrowMovement::add_to_right_foot(const double & time, const Struct3dRPY & position, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        sp_material_->add_point_to_right_foot(time, position, velocity, acceleration);

        if(debug_active_){
            right_foot_points_.at(0).emplace_back(bitbots_splines::Curve::Point{time, position.x_, velocity.x_, acceleration.x_});
            right_foot_points_.at(1).emplace_back(bitbots_splines::Curve::Point{time, position.y_, velocity.x_, acceleration.y_});
            right_foot_points_.at(2).emplace_back(bitbots_splines::Curve::Point{time, position.z_, velocity.x_, acceleration.z_});
            right_foot_points_.at(3).emplace_back(bitbots_splines::Curve::Point{time, position.roll_, velocity.x_, acceleration.roll_});
            right_foot_points_.at(4).emplace_back(bitbots_splines::Curve::Point{time, position.pitch_, velocity.pitch_, acceleration.pitch_});
            right_foot_points_.at(5).emplace_back(bitbots_splines::Curve::Point{time, position.yaw_, velocity.yaw_, acceleration.yaw_});
        }
    }
} //bitbots_throw