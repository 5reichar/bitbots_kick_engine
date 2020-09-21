#include "throws/throw_curves/throw_service.h"
#include "utility/throw_math.h"

namespace bitbots_throw{
    ThrowService::ThrowService(ThrowRequest request, ThrowType throw_type, ThrowEngineParameter engine_parameter)
        : request_(request)
        , throw_type_(throw_type)
        , engine_parameter_(engine_parameter)
        {
    }
    Struct3dRPY ThrowService::get_left_arm_start(){
        return request_.left_hand_position_;
    }
    Struct3dRPY ThrowService::get_left_arm_reach_to_ball(){
        Struct3dRPY point;
        point.x_ = request_.ball_position_.x_;
        point.y_ = request_.left_hand_position_.y_;
        point.z_ = request_.left_feet_position_.z_ + engine_parameter_.ball_radius_;
        point.roll_ = 0.0;
        point.pitch_ = 1.75 * engine_parameter_.pi_;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::get_left_arm_pick_up(){
        Struct3dRPY point;
        point.x_ = request_.ball_position_.x_;
        point.y_ = engine_parameter_.ball_radius_;
        point.z_ = request_.left_feet_position_.z_ + engine_parameter_.ball_radius_;
        point.roll_ = 0.0;
        point.pitch_ = 1.75 * engine_parameter_.pi_;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::get_left_arm_ball_at_head_height(){
        Struct3dRPY point;
        point.x_ = request_.ball_position_.x_;
        point.y_ = engine_parameter_.ball_radius_;
        point.z_ = request_.head_position_.z_ + engine_parameter_.arm_length_;
        point.roll_ = 0.0;
        point.pitch_ = engine_parameter_.pi_ / 2;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::get_left_arm_throw_zenith(){
        Struct3dRPY point;
        point.x_ = 0.0;
        point.y_ = engine_parameter_.ball_radius_;
        point.z_ = request_.head_position_.z_ + engine_parameter_.arm_length_;
        point.roll_ = 0.0;
        point.pitch_ = engine_parameter_.pi_ / 2;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::get_left_arm_throw_start(){
        Struct3dRPY point;
        point.x_ = -0.5 * engine_parameter_.arm_length_;
        point.y_ = engine_parameter_.ball_radius_;
        point.z_ = request_.head_position_.z_;
        point.roll_ = 0.0;
        point.pitch_ = engine_parameter_.pi_;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::get_left_arm_throw_release(){
        Struct3dRPY point = ThrowMath::calculate_throw_release_point(throw_type_.throw_angle_
                                                        ,engine_parameter_.arm_length_
                                                        ,0.0
                                                        ,request_.head_position_.z_ + engine_parameter_.arm_length_);
        point.y_ = engine_parameter_.ball_radius_;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_start(){
        return request_.right_hand_position_;
    }
    Struct3dRPY ThrowService::get_right_arm_reach_to_ball(){
        Struct3dRPY point;
        point.x_ = request_.ball_position_.x_;
        point.y_ = request_.right_hand_position_.y_;
        point.z_ = request_.right_feet_position_.z_ + engine_parameter_.ball_radius_;
        point.roll_ = 0.0;
        point.pitch_ = 1.75 * engine_parameter_.pi_;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_pick_up(){
        Struct3dRPY point;
        point.x_ = request_.ball_position_.x_;
        point.y_ = -1 * engine_parameter_.ball_radius_;
        point.z_ = request_.right_feet_position_.z_ + engine_parameter_.ball_radius_;
        point.roll_ = 0.0;
        point.pitch_ = 1.75 * engine_parameter_.pi_;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_ball_at_head_height(){
        Struct3dRPY point;
        point.x_ = request_.ball_position_.x_;
        point.y_ = -1 * engine_parameter_.ball_radius_;
        point.z_ = request_.head_position_.z_ + engine_parameter_.arm_length_;
        point.roll_ = 0.0;
        point.pitch_ = engine_parameter_.pi_ / 2;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_throw_zenith(){
        Struct3dRPY point;
        point.x_ = 0.0;
        point.y_ = -1 * engine_parameter_.ball_radius_;
        point.z_ = request_.head_position_.z_ + engine_parameter_.arm_length_;
        point.roll_ = 0.0;
        point.pitch_ = engine_parameter_.pi_ / 2;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_throw_start(){
        Struct3dRPY point;
        point.x_ = -0.5 * engine_parameter_.arm_length_;
        point.y_ = -1 * engine_parameter_.ball_radius_;
        point.z_ = request_.head_position_.z_;
        point.roll_ = 0.0;
        point.pitch_ = engine_parameter_.pi_;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_throw_release(){
        Struct3dRPY point = ThrowMath::calculate_throw_release_point(throw_type_.throw_angle_
                                                        ,engine_parameter_.arm_length_
                                                        ,0.0
                                                        ,request_.head_position_.z_ + engine_parameter_.arm_length_);
        point.y_ -= engine_parameter_.ball_radius_;
        point.roll_ = 0.0;
        point.pitch_ = 0.0;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::get_left_foot_start(){
        return request_.left_feet_position_;
    }
    Struct3dRPY ThrowService::get_left_foot_orientation_to_ball(){
        Struct3dRPY point = request_.left_feet_position_;
        if(request_.ball_position_.y_ != 0){
            point.pitch_ = ThrowMath::calculate_angle(request_.ball_position_);
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
        point.z_ = -1 * engine_parameter_.squat_safety_distance_;
        return point;
    }
    Struct3dRPY ThrowService::get_left_foot_orientation_to_goal(){
        Struct3dRPY point = request_.left_feet_position_;
        if(request_.goal_position_.y_ != 0){
            point.pitch_ = ThrowMath::calculate_angle(request_.goal_position_);
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
            point.pitch_ = ThrowMath::calculate_angle(request_.ball_position_);
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
        point.z_ = -1 * engine_parameter_.squat_safety_distance_;
        return point;
    }
    Struct3dRPY ThrowService::get_right_foot_orientation_to_goal(){
        Struct3dRPY point = request_.right_feet_position_;
        if(request_.goal_position_.y_ != 0){
            point.pitch_ = ThrowMath::calculate_angle(request_.goal_position_);
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
    Struct3dRPY ThrowService::get_throw_velocity(){
        Struct3dRPY point;
        point.x_ = engine_parameter_.gravity_ * request_.goal_position_.x_ / request_.goal_position_.z_;
        point.y_ = 0.0;
        point.z_ = 0.0;
        return point;
    }
}