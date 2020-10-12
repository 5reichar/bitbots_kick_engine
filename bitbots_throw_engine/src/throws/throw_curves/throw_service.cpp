#include "throws/throw_curves/throw_service.h"
#include "utility/throw_math.h"

namespace bitbots_throw{
    ThrowService::ThrowService(ThrowRequest request, ThrowType throw_type, RobotAndWorldParameter robot_and_world_parameter)
        : request_(request)
        , throw_type_(throw_type)
        , robot_and_world_parameter_(robot_and_world_parameter)
        {
    }
    Struct3dRPY ThrowService::get_left_arm_start(){
        return request_.left_hand_position_;
    }
    Struct3dRPY ThrowService::get_left_arm_reach_to_ball(){
        Struct3dRPY point = create_reach_to_ball();
        point.y_ = request_.left_hand_position_.y_;
        return point;
    }
    Struct3dRPY ThrowService::get_left_arm_pick_up(){
        return create_pick_up();
    }
    Struct3dRPY ThrowService::get_left_arm_ball_at_head_height(){
        return create_ball_at_head_height();
    }
    Struct3dRPY ThrowService::get_left_arm_throw_zenith(){
        return create_throw_zenith();
    }
    Struct3dRPY ThrowService::get_left_arm_throw_start(){
        return create_throw_start();
    }
    Struct3dRPY ThrowService::get_left_arm_throw_zenith_return(){
        return create_throw_zenith_return();
    }
    Struct3dRPY ThrowService::get_left_arm_throw_release(double const & angle_offset){
        Struct3dRPY point = create_throw_release(angle_offset);
        point.y_ = robot_and_world_parameter_.ball_radius_;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_start(){
        return request_.right_hand_position_;
    }
    Struct3dRPY ThrowService::get_right_arm_reach_to_ball(){
        Struct3dRPY point = create_reach_to_ball();
        point.y_ = request_.right_hand_position_.y_;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_pick_up(){
        Struct3dRPY point = create_pick_up();
        point.y_ *= -1;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_ball_at_head_height(){
        Struct3dRPY point = create_ball_at_head_height();
        point.y_ *= -1;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_throw_zenith(){
        Struct3dRPY point = create_throw_zenith();
        point.y_ *= -1;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_throw_start(){
        Struct3dRPY point = create_throw_start();
        point.y_ *= -1;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_throw_zenith_return(){
        Struct3dRPY point = create_throw_zenith_return();
        point.y_ *= -1;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_throw_release(double const & angle_offset){
        Struct3dRPY point = create_throw_release(angle_offset);
        point.y_ -= robot_and_world_parameter_.ball_radius_;
        return point;
    }
    Struct3dRPY ThrowService::get_left_foot_start(){
        return request_.left_feet_position_;
    }
    Struct3dRPY ThrowService::get_left_foot_orientation_to_ball(){
        Struct3dRPY point = request_.left_feet_position_;
        if(request_.ball_position_.y_ != 0){
            point.pitch_ = ThrowMath::calculate_angle(request_.ball_position_, robot_and_world_parameter_.pi_);
            //TODO: testing
            auto diffs = ThrowMath::calculate_foot_movement_for_rotate_robot(request_.ball_position_, point.y_);
            if(request_.ball_position_.y_ > 0){
                point.x_ -= diffs.first;
                point.y_ -= diffs.second;
                ThrowMath::calculate_positions_after_coordinate_rotation(point.pitch_, request_.goal_position_.x_, request_.goal_position_.y_, true);
            }else{
                point.x_ += diffs.first;
                point.y_ += diffs.second;
                ThrowMath::calculate_positions_after_coordinate_rotation(point.pitch_, request_.goal_position_.x_, request_.goal_position_.y_, false);
            }
        }
        return point;
    }
    Struct3dRPY ThrowService::get_left_foot_squat(){
        Struct3dRPY point = request_.left_feet_position_;
        point.z_ = -1 * robot_and_world_parameter_.squat_safety_distance_;
        return point;
    }
    Struct3dRPY ThrowService::get_left_foot_orientation_to_goal(){
        Struct3dRPY point = request_.left_feet_position_;
        if(request_.goal_position_.y_ != 0){
            point.pitch_ = ThrowMath::calculate_angle(request_.goal_position_, robot_and_world_parameter_.pi_);
            //TODO: testing
            auto diffs = ThrowMath::calculate_foot_movement_for_rotate_robot(request_.goal_position_, point.y_);
            if(request_.ball_position_.y_ > 0){
                point.x_ -= diffs.first;
                point.y_ -= diffs.second;
            }else{
                point.x_ += diffs.first;
                point.y_ += diffs.second;
            }
        }
        return point;
    }
    Struct3dRPY ThrowService::get_right_foot_start(){
        return request_.right_feet_position_;
    }
    Struct3dRPY ThrowService::get_right_foot_orientation_to_ball(){
        Struct3dRPY point = request_.right_feet_position_;
        if(request_.ball_position_.y_ != 0){
            point.pitch_ = ThrowMath::calculate_angle(request_.ball_position_, robot_and_world_parameter_.pi_);
            //TODO: testing
            auto diffs = ThrowMath::calculate_foot_movement_for_rotate_robot(request_.ball_position_, point.y_);
            if(request_.ball_position_.y_ > 0){
                point.x_ += diffs.first;
                point.y_ += diffs.second;
            }else{
                point.x_ -= diffs.first;
                point.y_ -= diffs.second;
            }
        }
        return point;
    }
    Struct3dRPY ThrowService::get_right_foot_squat(){
        Struct3dRPY point = request_.right_feet_position_;
        point.z_ = -1 * robot_and_world_parameter_.squat_safety_distance_;
        return point;
    }
    Struct3dRPY ThrowService::get_right_foot_orientation_to_goal(){
        Struct3dRPY point = request_.right_feet_position_;
        if(request_.goal_position_.y_ != 0){
            point.pitch_ = ThrowMath::calculate_angle(request_.goal_position_, robot_and_world_parameter_.pi_);
            //TODO: testing
            auto diffs = ThrowMath::calculate_foot_movement_for_rotate_robot(request_.goal_position_, point.y_);
            if(request_.ball_position_.y_ > 0){
                point.x_ += diffs.first;
                point.y_ += diffs.second;
            }else{
                point.x_ -= diffs.first;
                point.y_ -= diffs.second;
            }
        }
        return point;
    }
    double ThrowService::get_movement_time_pick_up_ball(){
        return throw_type_.movement_share_pick_up_ * throw_type_.movement_duration_;
    }
    double ThrowService::get_movement_time_throw_preparation(){
        return throw_type_.movement_share_preparation_ * throw_type_.movement_duration_;
    }
    double ThrowService::get_movement_time_throw(){
        return throw_type_.movement_share_throw_ * throw_type_.movement_duration_;
    }
    double ThrowService::get_movement_time_throw_conclusion(){
        return throw_type_.movement_share_conclusion_ * throw_type_.movement_duration_;
    }
    Struct3dRPY ThrowService::get_throw_velocity(double const & throw_release_z){
        Struct3dRPY point = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double distance = throw_release_z - request_.goal_position_.z_;
        double time_falling = std::sqrt(distance / robot_and_world_parameter_.gravity_);
        point.x_ = std::abs((request_.goal_position_.x_ / time_falling));
        return point;
    }
    Struct3dRPY ThrowService::create_reach_to_ball(){
        Struct3dRPY point;
        point.x_ = request_.ball_position_.x_;
        // Check for the case that one foot is not on the ground
        point.z_ = get_lowest_foot_position() + robot_and_world_parameter_.ball_radius_;
        point.roll_ = 0.0;
        point.pitch_ = 0.25 * robot_and_world_parameter_.pi_;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::create_pick_up(){
        Struct3dRPY point;
        point.x_ = request_.ball_position_.x_;
        point.y_ = robot_and_world_parameter_.ball_radius_;
        point.z_ = get_lowest_foot_position() + robot_and_world_parameter_.ball_radius_;
        point.roll_ = 0.0;
        point.pitch_ = 0.25 * robot_and_world_parameter_.pi_;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::create_ball_at_head_height(){
        Struct3dRPY point;
        point.x_ = request_.ball_position_.x_;
        point.y_ = robot_and_world_parameter_.ball_radius_;
        point.z_ = request_.head_position_.z_ + robot_and_world_parameter_.arm_length_;
        point.roll_ = 0.0;
        point.pitch_ = -0.5 * robot_and_world_parameter_.pi_;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::create_throw_zenith(){
        Struct3dRPY point;
        point.x_ = 0.0;
        point.y_ = robot_and_world_parameter_.ball_radius_;
        point.z_ = request_.head_position_.z_ + robot_and_world_parameter_.arm_length_;
        point.roll_ = 0.0;
        point.pitch_ = -0.5 * robot_and_world_parameter_.pi_;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::create_throw_start(){
        Struct3dRPY point;
        point.x_ = -0.85 * robot_and_world_parameter_.arm_length_;
        point.y_ = robot_and_world_parameter_.ball_radius_;
        point.z_ = request_.head_position_.z_ + robot_and_world_parameter_.head_height_ / 2;
        point.roll_ = 0.0;
        point.pitch_ = -0.9 * robot_and_world_parameter_.pi_;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::create_throw_zenith_return(){
        return create_throw_zenith();
    }
    Struct3dRPY ThrowService::create_throw_release(double const & angle_offset){
        return ThrowMath::calculate_throw_release_point(throw_type_.throw_angle_ - angle_offset
                                                       ,robot_and_world_parameter_.arm_length_
                                                       ,0.0
                                                       , request_.head_position_.z_ + robot_and_world_parameter_.arm_length_
                                                       ,robot_and_world_parameter_.pi_);
    }
    double ThrowService::get_lowest_foot_position(){
        double ret;
        if(request_.left_feet_position_.z_ < request_.right_feet_position_.z_){
            ret = request_.left_feet_position_.z_;
        }else{
            ret = request_.right_feet_position_.z_;
        }
        return ret;
    }
    void ThrowService::rotate_coordinate(const double & angle){

    }
    bool ThrowService::check_velocity(Struct3dRPY const & velocity){
        auto f = (robot_and_world_parameter_.arm_max_stall_torque_ * robot_and_world_parameter_.arm_stall_torque_usage_) / robot_and_world_parameter_.arm_length_;
        double max_velocity = f / robot_and_world_parameter_.ball_weight_;

        return max_velocity < velocity.x_ && max_velocity < velocity.y_ && max_velocity < velocity.z_;
    }
}