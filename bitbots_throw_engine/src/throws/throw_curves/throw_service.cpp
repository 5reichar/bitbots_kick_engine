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
    Struct3dRPY ThrowService::get_left_arm_pick_up(){
        auto pick_up_left_arm_ = request_.ball_position_;
        pick_up_left_arm_.y_ += engine_parameter_.ball_radius_;
        pick_up_left_arm_.z_ -= request_.left_feet_position_.z_;
        return pick_up_left_arm_;
    }
    Struct3dRPY ThrowService::get_left_arm_throw_zenith(){
        Struct3dRPY throw_zenith_left_arm_;
        throw_zenith_left_arm_.z_ = request_.head_position_.z_ + engine_parameter_.arm_length_;
        throw_zenith_left_arm_.y_ = get_left_arm_pick_up().y_;
        return throw_zenith_left_arm_;
    }
    Struct3dRPY ThrowService::get_left_arm_throw_start(){
        Struct3dRPY throw_start_left_arm_;
        throw_start_left_arm_.x_ = -0.5 * engine_parameter_.arm_length_;
        throw_start_left_arm_.y_ = get_left_arm_pick_up().y_;
        throw_start_left_arm_.z_ = request_.head_position_.z_;
        return throw_start_left_arm_;
    }
    Struct3dRPY ThrowService::get_left_arm_ball_at_head_height(){
        auto point = get_left_arm_pick_up();
        point.z_ = get_left_arm_throw_start().z_;
        return point;
    }
    Struct3dRPY ThrowService::get_left_arm_throw_release(){
        auto throw_release_left_arm_ = ThrowMath::calculate_throw_release_point(throw_type_.throw_angle_
                                                                    ,engine_parameter_.arm_length_
                                                                    ,ThrowMath::calculate_angle(request_.goal_position_)
                                                                    ,request_.head_position_.z_ + engine_parameter_.arm_length_);
        throw_release_left_arm_.y_ += engine_parameter_.ball_radius_;
        return throw_release_left_arm_;
    }
    Struct3dRPY ThrowService::get_right_arm_start(){
        return request_.right_hand_position_;
    }
    Struct3dRPY ThrowService::get_right_arm_pick_up(){
        auto pick_up_right_arm_ = request_.ball_position_;
        pick_up_right_arm_.y_ -= engine_parameter_.ball_radius_;
        pick_up_right_arm_.z_ -= request_.right_feet_position_.z_;
        return pick_up_right_arm_;
    }
    Struct3dRPY ThrowService::get_right_arm_throw_zenith(){
        Struct3dRPY throw_zenith_right_arm_;
        throw_zenith_right_arm_.z_ = request_.head_position_.z_ + engine_parameter_.arm_length_;
        throw_zenith_right_arm_.y_ = get_right_arm_pick_up().y_;
        return throw_zenith_right_arm_;
    }
    Struct3dRPY ThrowService::get_right_arm_throw_start(){
        auto throw_start_right_arm_ = get_left_arm_throw_start();
        throw_start_right_arm_.y_ = get_right_arm_pick_up().y_;
        return throw_start_right_arm_;
    }
    Struct3dRPY ThrowService::get_right_arm_ball_at_head_height(){
        auto point = get_right_arm_pick_up();
        point.z_ = get_right_arm_throw_start().z_;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_throw_release(){
        auto throw_release_right_arm_ = ThrowMath::calculate_throw_release_point(throw_type_.throw_angle_
                                                                                ,engine_parameter_.arm_length_
                                                                                ,ThrowMath::calculate_angle(request_.goal_position_)
                                                                                ,request_.head_position_.z_ + engine_parameter_.arm_length_);
        throw_release_right_arm_.y_ -= engine_parameter_.ball_radius_;
        return throw_release_right_arm_;
    }
    Struct3dRPY ThrowService::get_left_foot_start(){
        return request_.left_feet_position_;
    }
    Struct3dRPY ThrowService::get_left_foot_squat(){
        auto point = get_left_foot_start();
        point.z_ = -1 * engine_parameter_.squat_safety_distance_;
        return point;
    }
    Struct3dRPY ThrowService::get_right_foot_start(){
        return request_.right_feet_position_;
    }
    Struct3dRPY ThrowService::get_right_foot_squat(){
        auto point = get_right_foot_start();
        point.z_ = -1 * engine_parameter_.squat_safety_distance_;
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
}