#ifndef BITBOTS_THROW_THROW_TYPE_PARAMETER_BUILDER_H
#define BITBOTS_THROW_THROW_TYPE_PARAMETER_BUILDER_H

#include <memory>
#include "utility/throw_math.h"
#include "parameter/throw_type_parameter.h"
#include "parameter/throw_engine_parameter.h"
#include <bitbots_throw/throw_paramsConfig.h>

namespace bitbots_throw{
    class ThrowTypeParameterBuilder{
    public:
        static std::shared_ptr<ThrowTypeParameter> build_from_dynamic_reconf(bitbots_throw::throw_paramsConfig& config
                                                                            ,uint32_t level
                                                                            ,std::shared_ptr<RobotAndWorldParameter> params){
            auto sp_parameter = build_default();
            auto current_id = (ThrowTypeId)config.default_throw_enum;
            sp_parameter->default_throw_id_ = current_id;

            current_id = (ThrowTypeId)config.throw_1_throw_enum;
            sp_parameter->map_throw_types_[current_id] = build_throw_type(current_id
                                                                         ,config.throw_1_throw_active
                                                                         ,config.throw_1_throw_priority_level
                                                                         ,config.throw_1_throw_min_distance
                                                                         ,config.throw_1_throw_max_distance
                                                                         ,config.throw_1_throw_strength
                                                                         ,ThrowMath::degree_to_radian(config.throw_1_throw_angle, params->pi_)
                                                                         ,config.throw_1_goal_tolerance
                                                                         ,config.throw_1_velocity_adaptation_rate
                                                                         ,config.throw_1_movement_duration
                                                                         ,config.throw_1_movement_share_pick_up
                                                                         ,config.throw_1_movement_share_preparation
                                                                         ,config.throw_1_movement_share_throw
                                                                         ,config.throw_1_movement_share_conclusion
                                                                         ,config.throw_1_movement_offset_move_arms_away_from_ball);

            current_id = (ThrowTypeId)config.throw_2_throw_enum;
            sp_parameter->map_throw_types_[current_id] = build_throw_type(current_id
                                                                        ,config.throw_2_throw_active
                                                                        ,config.throw_2_throw_priority_level
                                                                        ,config.throw_2_throw_min_distance
                                                                        ,config.throw_2_throw_max_distance
                                                                        ,config.throw_2_throw_strength
                                                                        ,ThrowMath::degree_to_radian(config.throw_2_throw_angle, params->pi_)
                                                                        ,config.throw_2_goal_tolerance
                                                                        ,config.throw_2_velocity_adaptation_rate
                                                                        ,config.throw_2_movement_duration
                                                                        ,config.throw_2_movement_share_pick_up
                                                                        ,config.throw_2_movement_share_preparation
                                                                        ,config.throw_2_movement_share_throw
                                                                        ,config.throw_2_movement_share_conclusion
                                                                        ,config.throw_2_movement_offset_move_arms_away_from_ball);

            current_id = (ThrowTypeId)config.throw_3_throw_enum;
            sp_parameter->map_throw_types_[current_id] = build_throw_type(current_id
                                                                        ,config.throw_3_throw_active
                                                                        ,config.throw_3_throw_priority_level
                                                                        ,config.throw_3_throw_min_distance
                                                                        ,config.throw_3_throw_max_distance
                                                                        ,config.throw_3_throw_strength
                                                                        ,ThrowMath::degree_to_radian(config.throw_3_throw_angle, params->pi_)
                                                                        ,config.throw_3_goal_tolerance
                                                                        ,config.throw_3_velocity_adaptation_rate
                                                                        ,config.throw_3_movement_duration
                                                                        ,config.throw_3_movement_share_pick_up
                                                                        ,config.throw_3_movement_share_preparation
                                                                        ,config.throw_3_movement_share_throw
                                                                        ,config.throw_3_movement_share_conclusion
                                                                        ,config.throw_3_movement_offset_move_arms_away_from_ball);

            current_id = (ThrowTypeId)config.throw_4_throw_enum;
            sp_parameter->map_throw_types_[current_id] = build_throw_type(current_id
                                                                        ,config.throw_4_throw_active
                                                                        ,config.throw_4_throw_priority_level
                                                                        ,config.throw_4_throw_min_distance
                                                                        ,config.throw_4_throw_max_distance
                                                                        ,config.throw_4_throw_strength
                                                                        ,ThrowMath::degree_to_radian(config.throw_4_throw_angle, params->pi_)
                                                                        ,config.throw_4_goal_tolerance
                                                                        ,config.throw_4_velocity_adaptation_rate
                                                                        ,config.throw_4_movement_duration
                                                                        ,config.throw_4_movement_share_pick_up
                                                                        ,config.throw_4_movement_share_preparation
                                                                        ,config.throw_4_movement_share_throw
                                                                        ,config.throw_4_movement_share_conclusion
                                                                        ,config.throw_4_movement_offset_move_arms_away_from_ball);

            sp_parameter->map_throw_types_[ThrowTypeId::none] = build_throw_type((ThrowTypeId)config.default_throw_enum
                                                                          ,true
                                                                          ,0
                                                                          ,0.0
                                                                          ,0.0
                                                                          ,config.throw_strength
                                                                          ,ThrowMath::degree_to_radian(config.throw_angle, params->pi_)
                                                                          ,config.goal_tolerance
                                                                          ,config.velocity_adaptation_rate
                                                                          ,config.movement_duration
                                                                          ,config.movement_share_pick_up
                                                                          ,config.movement_share_preparation
                                                                          ,config.movement_share_throw
                                                                          ,config.movement_share_conclusion
                                                                          ,config.movement_offset_move_arms_away_from_ball);

            return sp_parameter;
        };

        static std::shared_ptr<ThrowTypeParameter> build_default(){
            return std::make_shared<ThrowTypeParameter>(ThrowTypeId::smooth_spline);
        };

    protected:
        static std::shared_ptr<ThrowType> build_throw_type(ThrowTypeId id
                                                          ,bool active
                                                          ,int priority_level
                                                          ,double min_throw_distance
                                                          ,double max_throw_distance
                                                          ,double throw_strength
                                                          ,double throw_angle
                                                          ,double goal_tolerance
                                                          ,double velocity_adaptation_rate
                                                          ,double movement_duration
                                                          ,double movement_share_pick_up
                                                          ,double movement_share_preparation
                                                          ,double movement_share_throw
                                                          ,double movement_share_conclusion
                                                          ,double movement_offset_move_arms_away_from_ball){
            return std::make_shared<ThrowType>(id
                                              ,active
                                              ,priority_level
                                              ,min_throw_distance
                                              ,max_throw_distance
                                              ,throw_strength
                                              ,throw_angle
                                              ,goal_tolerance
                                              ,velocity_adaptation_rate
                                              ,movement_duration
                                              ,movement_share_pick_up
                                              ,movement_share_preparation
                                              ,movement_share_throw
                                              ,movement_share_conclusion
                                              ,movement_offset_move_arms_away_from_ball);
        };
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_TYPE_PARAMETER_BUILDER_H