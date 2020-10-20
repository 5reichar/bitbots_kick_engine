#include "throws/throw_curves/throw_movement_base.h"
#include <utility>

namespace bitbots_throw{
    ThrowMovementBase::ThrowMovementBase(std::shared_ptr<ThrowMaterial> material){
        sp_material_ = std::move(material);
        trajectory_time_ = 0.0;
        debug_active_ = false;
    }

    void ThrowMovementBase::init(std::shared_ptr<ThrowService> service){
        sp_service_ = std::move(service);
    }

    std::shared_ptr<ThrowMaterial> ThrowMovementBase::create_material(bool debug_active){
        reset_debug_data(debug_active);
        trajectory_time_ = 0.0;
        add_movement();
        return sp_material_;
    }

    std::vector<std::vector<bitbots_splines::Curve::Point>> ThrowMovementBase::get_left_hand_points(){
        return left_hand_points_;
    }

    std::vector<std::vector<bitbots_splines::Curve::Point>> ThrowMovementBase::get_right_hand_points(){
        return right_hand_points_;
    }

    std::vector<std::vector<bitbots_splines::Curve::Point>> ThrowMovementBase::get_left_foot_points(){
        return left_foot_points_;
    }

    std::vector<std::vector<bitbots_splines::Curve::Point>> ThrowMovementBase::get_right_foot_points(){
        return right_foot_points_;
    }

    void ThrowMovementBase::reset_debug_data(bool const & debug_active){
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

    void ThrowMovementBase::add_to_left_hand(const Struct3dRPY & position, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        sp_material_->add_point_to_left_hand(trajectory_time_, position, velocity, acceleration);
        add_point_to_debug(left_hand_points_, trajectory_time_, position, velocity, acceleration);
    }

    void ThrowMovementBase::add_to_right_hand(const Struct3dRPY & position, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        sp_material_->add_point_to_right_hand(trajectory_time_, position, velocity, acceleration);
        add_point_to_debug(right_hand_points_, trajectory_time_, position, velocity, acceleration);
    }

    void ThrowMovementBase::add_to_left_foot(const Struct3dRPY & position, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        sp_material_->add_point_to_left_foot(trajectory_time_, position, velocity, acceleration);
        add_point_to_debug(left_foot_points_, trajectory_time_, position, velocity, acceleration);
    }

    void ThrowMovementBase::add_to_right_foot(const Struct3dRPY & position, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        sp_material_->add_point_to_right_foot(trajectory_time_, position, velocity, acceleration);
        add_point_to_debug(right_foot_points_, trajectory_time_, position, velocity, acceleration);
    }

    void ThrowMovementBase::add_point_to_debug(std::vector<std::vector<bitbots_splines::Curve::Point>> & debug_points
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