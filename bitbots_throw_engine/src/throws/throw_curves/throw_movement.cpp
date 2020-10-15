#include "throws/throw_curves/throw_movement.h"

#include <utility>

namespace bitbots_throw{
    ThrowMovement::ThrowMovement(std::shared_ptr<ThrowMaterial> material){
        sp_material_ = std::move(material);
        trajectory_time_ = 0.0;
        debug_active_ = false;
    }

    void ThrowMovement::init(std::shared_ptr<ThrowService> service){
        sp_service_ = std::move(service);
    }

    std::shared_ptr<ThrowMaterial> ThrowMovement::create_material(bool debug_active){
        reset_debug_data(debug_active);
        trajectory_time_ = 0.0;
        add_movement();
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

    void ThrowMovement::reset_debug_data(bool const & debug_active){
        debug_active_ = debug_active;
        left_hand_points_.clear();
        left_hand_points_.resize(6);
        right_hand_points_.clear();
        right_hand_points_.resize(6);
        left_foot_points_.clear();
        left_foot_points_.resize(6);
        right_foot_points_.clear();
        right_foot_points_.resize(6);
    }

    void ThrowMovement::add_movement(){
        add_movement_starting_position();

        auto movement_time = sp_service_->get_movement_time_pick_up_ball() / 4;
        trajectory_time_ += movement_time;
        add_movement_orient_to_ball();
        trajectory_time_ += movement_time;
        add_movement_squat();
        trajectory_time_ += movement_time;
        add_movement_reach_to_ball();
        trajectory_time_ += movement_time;
        add_movement_pick_ball();

        movement_time = sp_service_->get_movement_time_throw_preparation() / 2;
        trajectory_time_ += movement_time;
        add_movement_stand_up();
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
        add_to_left_foot(sp_service_->get_left_foot_squat());
        add_to_right_foot(sp_service_->get_right_foot_squat());
    }

    void ThrowMovement::add_movement_reach_to_ball(){
        add_to_left_hand(sp_service_->get_left_arm_reach_to_ball());
        add_to_right_hand(sp_service_->get_right_arm_reach_to_ball());
        add_to_left_foot(sp_service_->get_left_foot_squat());
        add_to_right_foot(sp_service_->get_right_foot_squat());
    }

    void ThrowMovement::add_movement_pick_ball(){
        add_to_left_hand(sp_service_->get_left_arm_pick_up());
        add_to_right_hand(sp_service_->get_right_arm_pick_up());
        add_to_left_foot(sp_service_->get_left_foot_squat());
        add_to_right_foot(sp_service_->get_right_foot_squat());
    }

    void ThrowMovement::add_movement_stand_up(){
        add_to_left_hand(sp_service_->get_left_arm_ball_at_head_height());
        add_to_right_hand(sp_service_->get_right_arm_ball_at_head_height());
        add_to_left_foot(sp_service_->get_left_foot_start());
        add_to_right_foot(sp_service_->get_right_foot_start());
    }

    void ThrowMovement::add_movement_prepare_throw(){
        add_to_left_hand(sp_service_->get_left_arm_throw_start());
        add_to_right_hand(sp_service_->get_right_arm_throw_start());
        add_to_left_foot(sp_service_->get_left_foot_orientation_to_goal());
        add_to_right_foot(sp_service_->get_right_foot_orientation_to_goal());
    }

    void ThrowMovement::add_movement_throw(){
        // get Values
        double duration_throw = sp_service_->get_movement_time_throw();
        duration_throw -= sp_service_->get_movement_offset_move_arms_away_from_ball();
        auto left_arm_throw_release = sp_service_->get_left_arm_throw_release();
        Struct3dRPY velocity = sp_service_->calculate_throw_velocity(duration_throw);

        ////  Movement
        ////==== Release ball
        trajectory_time_ -= sp_service_->get_movement_offset_move_arms_away_from_ball();
        add_to_left_hand( left_arm_throw_release, velocity, velocity/duration_throw);
        add_to_right_hand(sp_service_->get_right_arm_throw_release(), velocity, velocity/duration_throw);

        ////==== Move arms away from the ball
        trajectory_time_ += sp_service_->get_movement_offset_move_arms_away_from_ball();
        add_to_left_hand(sp_service_->get_left_arm_move_away_from_ball());
        add_to_right_hand(sp_service_->get_right_arm_move_away_from_ball());
    }

    void ThrowMovement::add_movement_return_to_starting_position(){
        add_to_left_hand(sp_service_->get_left_arm_start());
        add_to_right_hand(sp_service_->get_right_arm_start());
    }

    void ThrowMovement::add_to_left_hand(const Struct3dRPY & position, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        sp_material_->add_point_to_left_hand(trajectory_time_, position, velocity, acceleration);
        add_point_to_debug(left_hand_points_, trajectory_time_, position, velocity, acceleration);
    }

    void ThrowMovement::add_to_right_hand(const Struct3dRPY & position, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        sp_material_->add_point_to_right_hand(trajectory_time_, position, velocity, acceleration);
        add_point_to_debug(right_hand_points_, trajectory_time_, position, velocity, acceleration);
    }

    void ThrowMovement::add_to_left_foot(const Struct3dRPY & position, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        sp_material_->add_point_to_left_foot(trajectory_time_, position, velocity, acceleration);
        add_point_to_debug(left_foot_points_, trajectory_time_, position, velocity, acceleration);
    }

    void ThrowMovement::add_to_right_foot(const Struct3dRPY & position, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        sp_material_->add_point_to_right_foot(trajectory_time_, position, velocity, acceleration);
        add_point_to_debug(right_foot_points_, trajectory_time_, position, velocity, acceleration);
    }

    void ThrowMovement::add_point_to_debug(std::vector<std::vector<bitbots_splines::Curve::Point>> & debug_points
                                           , const double & time
                                           , const Struct3dRPY & position
                                           , const Struct3dRPY & velocity
                                           , const Struct3dRPY & acceleration){
        if(debug_active_){
            debug_points.at(0).emplace_back(bitbots_splines::Curve::Point{time, position.x_, velocity.x_, acceleration.x_});
            debug_points.at(1).emplace_back(bitbots_splines::Curve::Point{time, position.y_, velocity.x_, acceleration.y_});
            debug_points.at(2).emplace_back(bitbots_splines::Curve::Point{time, position.z_, velocity.x_, acceleration.z_});
            debug_points.at(3).emplace_back(bitbots_splines::Curve::Point{time, position.roll_, velocity.x_, acceleration.roll_});
            debug_points.at(4).emplace_back(bitbots_splines::Curve::Point{time, position.pitch_, velocity.pitch_, acceleration.pitch_});
            debug_points.at(5).emplace_back(bitbots_splines::Curve::Point{time, position.yaw_, velocity.yaw_, acceleration.yaw_});
        }
    }
} //bitbots_throw