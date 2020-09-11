#include "throws/throw_curves/throw_service.h"

namespace bitbots_throw{

    Struct3dRPY ThrowService::get_left_arm_start(){
        return throw_parameter_.start_left_arm_;
    }
    Struct3dRPY ThrowService::get_left_arm_pick_up(){
        return throw_parameter_.pick_up_left_arm_;
    }
    Struct3dRPY ThrowService::get_left_arm_throw_zenith(){
        return throw_parameter_.throw_zenith_left_arm_;
    }
    Struct3dRPY ThrowService::get_left_arm_throw_start(){
        return throw_parameter_.throw_start_left_arm_;
    }
    Struct3dRPY ThrowService::get_left_arm_ball_at_head_height(){
        auto point = get_left_arm_pick_up();
        point.z_ = get_left_arm_throw_start().z_;
        return point;
    }
    Struct3dRPY ThrowService::get_left_arm_throw_release(){
        return throw_parameter_.throw_release_left_arm_;
    }
    Struct3dRPY ThrowService::get_right_arm_start(){
        return throw_parameter_.start_right_arm_;
    }
    Struct3dRPY ThrowService::get_right_arm_pick_up(){
        return throw_parameter_.pick_up_right_arm_;
    }
    Struct3dRPY ThrowService::get_right_arm_throw_zenith(){
        return throw_parameter_.throw_zenith_right_arm_;
    }
    Struct3dRPY ThrowService::get_right_arm_throw_start(){
        return throw_parameter_.throw_start_right_arm_;
    }
    Struct3dRPY ThrowService::get_right_arm_ball_at_head_height(){
        auto point = get_right_arm_pick_up();
        point.z_ = get_right_arm_throw_start().z_;
        return point;
    }
    Struct3dRPY ThrowService::get_right_arm_throw_release(){
        return throw_parameter_.throw_release_right_arm_;
    }
    Struct3dRPY ThrowService::get_left_foot_start(){
        return throw_parameter_.start_left_feet_;
    }
    Struct3dRPY ThrowService::get_left_foot_squat(){
        auto point = get_left_foot_start();
        point.z_ = throw_parameter_.pick_up_trunk_.z_ - throw_parameter_.squat_safety_distance_;
        return point;
    }
    Struct3dRPY ThrowService::get_right_foot_start(){
        return throw_parameter_.start_right_feet_;
    }
    Struct3dRPY ThrowService::get_right_foot_squat(){
        auto point = get_right_foot_start();
        point.z_ = throw_parameter_.pick_up_trunk_.z_ - throw_parameter_.squat_safety_distance_;
        return point;
    }
}