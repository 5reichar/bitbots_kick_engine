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
        Struct3dRPY point = create_pick_up();
        point.y_ += robot_and_world_parameter_.ball_radius_;
        return point;
    }
    Struct3dRPY ThrowService::get_left_arm_ball_at_head_height(){
        Struct3dRPY point = create_ball_at_head_height();
        point.y_ += robot_and_world_parameter_.ball_radius_;
        return point;
    }
    Struct3dRPY ThrowService::get_left_arm_throw_start(){
        Struct3dRPY point = create_throw_start();
        point.y_ += robot_and_world_parameter_.ball_radius_;
        return point;
    }
    Struct3dRPY ThrowService::get_left_arm_throw_release(){
        Struct3dRPY point = create_throw_release();
        point.y_ += robot_and_world_parameter_.ball_radius_;
        return point;
    }
    Struct3dRPY ThrowService::get_left_arm_move_away_from_ball(Struct3dRPY const & velocity){
        Struct3dRPY point = get_left_arm_throw_release();
        point.x_ += velocity.x_ * get_movement_offset_move_arms_away_from_ball();
        point.y_ += robot_and_world_parameter_.move_arms_away_from_ball_;
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
        point.y_ -= robot_and_world_parameter_.ball_radius_;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_ball_at_head_height(){
        Struct3dRPY point = create_ball_at_head_height();
        point.y_ -= robot_and_world_parameter_.ball_radius_;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_throw_start(){
        Struct3dRPY point = create_throw_start();
        point.y_ -= robot_and_world_parameter_.ball_radius_;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_throw_release(){
        Struct3dRPY point = create_throw_release();
        point.y_ -= robot_and_world_parameter_.ball_radius_;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_move_away_from_ball(Struct3dRPY const & velocity){
        Struct3dRPY point = get_left_arm_throw_release();
        point.x_ += velocity.x_ * get_movement_offset_move_arms_away_from_ball();
        point.y_ -= robot_and_world_parameter_.move_arms_away_from_ball_;
        return point;
    }
    Struct3dRPY ThrowService::get_left_foot_start(){
        return request_.left_feet_position_;
    }
    Struct3dRPY ThrowService::get_left_foot_orientation_to_ball(){
        Struct3dRPY point = request_.left_feet_position_ + create_orientation_to_ball(true, true);
        return point;
    }
    Struct3dRPY ThrowService::get_left_foot_squat(){
        Struct3dRPY point = request_.left_feet_position_;
        point.z_ = -1 * robot_and_world_parameter_.squat_safety_distance_;
        point.pitch_ = -1 * calculate_squat_bow_angle();
        return point;
    }
    Struct3dRPY ThrowService::get_left_foot_orientation_to_goal(){
        Struct3dRPY point = request_.left_feet_position_ + create_orientation_to_goal(true);
        return point;
    }
    Struct3dRPY ThrowService::get_right_foot_start(){
        return request_.right_feet_position_;
    }
    Struct3dRPY ThrowService::get_right_foot_orientation_to_ball(){
        Struct3dRPY point = request_.right_feet_position_ + create_orientation_to_ball(false, false);
        return point;
    }
    Struct3dRPY ThrowService::get_right_foot_squat(){
        Struct3dRPY point = request_.right_feet_position_;
        point.z_ = -1 * robot_and_world_parameter_.squat_safety_distance_;
        point.pitch_ = -1 * calculate_squat_bow_angle();
        return point;
    }
    Struct3dRPY ThrowService::get_right_foot_orientation_to_goal(){
        Struct3dRPY point = request_.right_feet_position_ + create_orientation_to_goal(false);
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
    double ThrowService::get_movement_offset_move_arms_away_from_ball(){
        return throw_type_.movement_offset_move_arms_away_from_ball_;
    }
    Struct3dRPY ThrowService::calculate_throw_velocity(double const & duration_build_up_velocity){
        double const height = request_.head_position_.z_ + robot_and_world_parameter_.arm_length_;
        double const goal_distance = ThrowMath::calculate_distance(request_.goal_position_);
        double const max_velocity = calculate_robot_max_velocity(duration_build_up_velocity);
        double reference_velocity =  max_velocity * throw_type_.throw_strength_;
        enum adapt_status : int8_t {init=0, adding, subtracting};
        adapt_status status = adapt_status::init;
        double diff = 0.0;
        bool kill_switch = false;

        do{
            double time_ball_reach_goal = ((2 * reference_velocity * std::sin(throw_type_.throw_angle_))
                                           + std::sqrt(
                    std::pow(reference_velocity * std::sin(throw_type_.throw_angle_), 2)
                    + 2 * height * robot_and_world_parameter_.gravity_))
                                          / robot_and_world_parameter_.gravity_;
            double s_x = reference_velocity * std::cos(throw_type_.throw_angle_) * time_ball_reach_goal;

            diff = goal_distance - s_x;

            if(diff < throw_type_.goal_tolerance_){
                reference_velocity -= throw_type_.velocity_adaptation_rate_;

                // part to avoid inf. loop
                if(adapt_status::init == status){
                    status = adapt_status::adding;
                }else if(adapt_status::subtracting == status){
                    kill_switch = true;
                }
            }else if(diff > throw_type_.goal_tolerance_){
                reference_velocity += throw_type_.velocity_adaptation_rate_;

                // part to avoid inf. loop
                if(adapt_status::init == status){
                    status = adapt_status::subtracting;
                }else if(adapt_status::adding == status){
                    kill_switch = true;
                }
            }else{
                // velocity found
                kill_switch = true;
            }
        }while(kill_switch && reference_velocity <= max_velocity && reference_velocity > 0);

        return Struct3dRPY(reference_velocity * std::cos(throw_type_.throw_angle_)
                           ,0.0
                           ,reference_velocity * std::sin(throw_type_.throw_angle_)
                           ,0.0
                           ,0.0
                           ,0.0);
    }
    double ThrowService::calculate_robot_max_velocity(double const & duration_build_up_velocity) const{
        auto f = (robot_and_world_parameter_.arm_max_stall_torque_ * robot_and_world_parameter_.arm_stall_torque_usage_) / robot_and_world_parameter_.arm_length_;
        double max_acceleration = f / robot_and_world_parameter_.ball_weight_;
        double max_velocity = max_acceleration * duration_build_up_velocity;
        return max_velocity;
    }
    Struct3dRPY ThrowService::create_reach_to_ball(){
        Struct3dRPY point;
        point.x_ = request_.ball_position_.x_;
        point.y_ = 0.0;
        point.z_ = get_lowest_foot_position() + robot_and_world_parameter_.ball_radius_;
        point.roll_ = 0.0;
        point.pitch_ = 0.25 * robot_and_world_parameter_.pi_;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::create_pick_up(){
        Struct3dRPY point;
        point.x_ = request_.ball_position_.x_;
        point.y_ = 0.0;
        point.z_ = get_lowest_foot_position() + robot_and_world_parameter_.ball_radius_;
        point.roll_ = 0.0;
        point.pitch_ = 0.25 * robot_and_world_parameter_.pi_;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::create_ball_at_head_height(){
        Struct3dRPY point;
        point.x_ = request_.ball_position_.x_;
        point.y_ = 0.0;
        point.z_ = request_.head_position_.z_ + robot_and_world_parameter_.arm_length_;
        point.roll_ = 0.0;
        point.pitch_ = -0.5 * robot_and_world_parameter_.pi_;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::create_throw_start(){
        return ThrowMath::calculate_throw_start_point(throw_type_.throw_angle_
                                                     ,request_.head_position_.z_ + robot_and_world_parameter_.arm_length_
                                                     ,robot_and_world_parameter_.pi_);
    }
    Struct3dRPY ThrowService::create_throw_release(){
        Struct3dRPY point;
        point.x_ = 0.0;
        point.y_ = 0.0;
        point.z_ = request_.head_position_.z_ + robot_and_world_parameter_.arm_length_;
        point.roll_ = 0.0;
        point.pitch_ = -0.5 * robot_and_world_parameter_.pi_;
        point.yaw_ = 0.0;
        return point;
    }
    Struct3dRPY ThrowService::create_orientation_to_ball(const bool & left_foot, const bool & rotate_coordinate){
        Struct3dRPY point;
        if(request_.ball_position_.y_ != 0){
            bool rotate_left = true;
            point.pitch_ = ThrowMath::calculate_angle(request_.ball_position_, robot_and_world_parameter_.pi_);

            //TODO: testing
            auto diffs = ThrowMath::calculate_foot_movement_for_rotate_robot(request_.ball_position_, point.y_);
            if(left_foot ^ (request_.ball_position_.y_ > 0)){
                point.x_ -= diffs.first;
                point.y_ -= diffs.second;
            }else{
                point.x_ += diffs.first;
                point.y_ += diffs.second;
                rotate_left = false;
            }

            if(rotate_coordinate){
                ThrowMath::calculate_positions_after_coordinate_rotation(point.pitch_, request_.goal_position_.x_, request_.goal_position_.y_, rotate_left);
            }
        }

        return point;
    }
    Struct3dRPY ThrowService::create_orientation_to_goal(const bool & left_foot){
        Struct3dRPY point;
        if(request_.goal_position_.y_ != 0){
            point.pitch_ = ThrowMath::calculate_angle(request_.goal_position_, robot_and_world_parameter_.pi_);
            //TODO: testing
            auto diffs = ThrowMath::calculate_foot_movement_for_rotate_robot(request_.goal_position_, point.y_);
            if(left_foot ^ (request_.ball_position_.y_ > 0)){
                point.x_ -= diffs.first;
                point.y_ -= diffs.second;
            }else{
                point.x_ += diffs.first;
                point.y_ += diffs.second;
            }
        }
        return point;
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
    double ThrowService::calculate_squat_bow_angle(){
        auto distance_to_ball = ThrowMath::calculate_distance(request_.ball_position_);
        auto squat_height = robot_and_world_parameter_.squat_safety_distance_ + robot_and_world_parameter_.trunk_height_;
        auto beta = std::acos((pow(robot_and_world_parameter_.arm_length_, 2) - pow(squat_height, 2) - pow(distance_to_ball, 2)) / (-2 * squat_height * distance_to_ball));
        return beta;
    }
}