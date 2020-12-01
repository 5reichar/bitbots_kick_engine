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
        static std::map<ThrowTypeId, std::shared_ptr<ThrowType>> build_from_dynamic_reconf(bitbots_throw::throw_paramsConfig& config
                                                                            ,uint32_t level
                                                                            ,std::shared_ptr<RobotAndWorldParameter> params){
            std::map<ThrowTypeId, std::shared_ptr<ThrowType>> throw_type_map;

            throw_type_map[ThrowTypeId::none] = std::make_shared<ThrowType>(true
                                                                            ,0
                                                                            ,(ThrowMovementId)config.throw_movement_enum
                                                                            ,(ThrowCurveId)config.arms_curve_enum
                                                                            ,(ThrowCurveId)config.legs_curve_enum
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

                config.throw_1_throw_angle_default == 0 ? throw_type_map[ThrowTypeId::none]->throw_angle_ : ThrowMath::degree_to_radian(config.throw_1_throw_angle, params->pi_);
            throw_type_map[ThrowTypeId::throw_1] = std::make_shared<ThrowType>(config.throw_1_throw_active
                                                                              ,config.throw_1_throw_priority_level
                                                                              ,(ThrowMovementId)config.throw_1_throw_movement_enum
                                                                              ,(ThrowCurveId)config.throw_1_arms_curve_enum
                                                                              ,(ThrowCurveId)config.throw_1_legs_curve_enum
                                                                              ,config.throw_1_throw_min_distance
                                                                              ,config.throw_1_throw_max_distance
                                                                              ,config.throw_1_throw_strength == 0 ? throw_type_map[ThrowTypeId::none]->throw_strength_ : config.throw_1_throw_strength
                                                                              ,config.throw_1_throw_angle_default == 0 ? throw_type_map[ThrowTypeId::none]->throw_angle_ : ThrowMath::degree_to_radian(config.throw_1_throw_angle, params->pi_)
                                                                              ,config.throw_1_goal_tolerance
                                                                              ,config.throw_1_velocity_adaptation_rate
                                                                              ,config.throw_1_movement_duration == 0 ? throw_type_map[ThrowTypeId::none]->movement_duration_ : config.throw_1_movement_duration
                                                                              ,config.throw_1_movement_share_pick_up == 0 ? throw_type_map[ThrowTypeId::none]->movement_share_pick_up_ : config.throw_1_movement_share_pick_up
                                                                              ,config.throw_1_movement_share_preparation == 0 ? throw_type_map[ThrowTypeId::none]->movement_share_preparation_ : config.throw_1_movement_share_preparation
                                                                              ,config.throw_1_movement_share_throw == 0 ? throw_type_map[ThrowTypeId::none]->movement_share_throw_ : config.throw_1_movement_share_throw
                                                                              ,config.throw_1_movement_share_conclusion == 0 ? throw_type_map[ThrowTypeId::none]->movement_share_conclusion_ : config.throw_1_movement_share_conclusion
                                                                              ,config.throw_1_movement_offset_move_arms_away_from_ball == 0 ? throw_type_map[ThrowTypeId::none]->movement_offset_move_arms_away_from_ball_ : config.throw_1_movement_offset_move_arms_away_from_ball);

            throw_type_map[ThrowTypeId::throw_2] = std::make_shared<ThrowType>(config.throw_2_throw_active
                                                                              ,config.throw_2_throw_priority_level
                                                                              ,(ThrowMovementId)config.throw_2_throw_movement_enum
                                                                              ,(ThrowCurveId)config.throw_2_arms_curve_enum
                                                                              ,(ThrowCurveId)config.throw_2_legs_curve_enum
                                                                              ,config.throw_2_throw_min_distance
                                                                              ,config.throw_2_throw_max_distance
                                                                              ,config.throw_2_throw_strength == 0 ? throw_type_map[ThrowTypeId::none]->throw_strength_ : config.throw_2_throw_strength
                                                                              ,config.throw_2_throw_angle_default == 0 ? throw_type_map[ThrowTypeId::none]->throw_angle_ : ThrowMath::degree_to_radian(config.throw_2_throw_angle, params->pi_)
                                                                              ,config.throw_2_goal_tolerance
                                                                              ,config.throw_2_velocity_adaptation_rate
                                                                              ,config.throw_2_movement_duration == 0 ? throw_type_map[ThrowTypeId::none]->movement_duration_ : config.throw_2_movement_duration
                                                                              ,config.throw_2_movement_share_pick_up == 0 ? throw_type_map[ThrowTypeId::none]->movement_share_pick_up_ : config.throw_2_movement_share_pick_up
                                                                              ,config.throw_2_movement_share_preparation == 0 ? throw_type_map[ThrowTypeId::none]->movement_share_preparation_ : config.throw_2_movement_share_preparation
                                                                              ,config.throw_2_movement_share_throw == 0 ? throw_type_map[ThrowTypeId::none]->movement_share_throw_ : config.throw_2_movement_share_throw
                                                                              ,config.throw_2_movement_share_conclusion == 0 ? throw_type_map[ThrowTypeId::none]->movement_share_conclusion_ : config.throw_2_movement_share_conclusion
                                                                              ,config.throw_2_movement_offset_move_arms_away_from_ball == 0 ? throw_type_map[ThrowTypeId::none]->movement_offset_move_arms_away_from_ball_ : config.throw_2_movement_offset_move_arms_away_from_ball);

            throw_type_map[ThrowTypeId::throw_3] = std::make_shared<ThrowType>(config.throw_3_throw_active
                                                                              ,config.throw_3_throw_priority_level
                                                                              ,(ThrowMovementId)config.throw_3_throw_movement_enum
                                                                              ,(ThrowCurveId)config.throw_3_arms_curve_enum
                                                                              ,(ThrowCurveId)config.throw_3_legs_curve_enum
                                                                              ,config.throw_3_throw_min_distance
                                                                              ,config.throw_3_throw_max_distance
                                                                              ,config.throw_3_throw_strength == 0 ? throw_type_map[ThrowTypeId::none]->throw_strength_ : config.throw_3_throw_strength
                                                                              ,config.throw_3_throw_angle_default == 0 ? throw_type_map[ThrowTypeId::none]->throw_angle_ : ThrowMath::degree_to_radian(config.throw_3_throw_angle, params->pi_)
                                                                              ,config.throw_3_goal_tolerance
                                                                              ,config.throw_3_velocity_adaptation_rate
                                                                              ,config.throw_3_movement_duration == 0 ? throw_type_map[ThrowTypeId::none]->movement_duration_ : config.throw_3_movement_duration
                                                                              ,config.throw_3_movement_share_pick_up == 0 ? throw_type_map[ThrowTypeId::none]->movement_share_pick_up_ : config.throw_3_movement_share_pick_up
                                                                              ,config.throw_3_movement_share_preparation == 0 ? throw_type_map[ThrowTypeId::none]->movement_share_preparation_ : config.throw_3_movement_share_preparation
                                                                              ,config.throw_3_movement_share_throw == 0 ? throw_type_map[ThrowTypeId::none]->movement_share_throw_ : config.throw_3_movement_share_throw
                                                                              ,config.throw_3_movement_share_conclusion == 0 ? throw_type_map[ThrowTypeId::none]->movement_share_conclusion_ : config.throw_3_movement_share_conclusion
                                                                              ,config.throw_3_movement_offset_move_arms_away_from_ball == 0 ? throw_type_map[ThrowTypeId::none]->movement_offset_move_arms_away_from_ball_ : config.throw_3_movement_offset_move_arms_away_from_ball);

            throw_type_map[ThrowTypeId::throw_4] = std::make_shared<ThrowType>(config.throw_4_throw_active
                                                                              ,config.throw_4_throw_priority_level
                                                                              ,(ThrowMovementId)config.throw_4_throw_movement_enum
                                                                              ,(ThrowCurveId)config.throw_4_arms_curve_enum
                                                                              ,(ThrowCurveId)config.throw_4_legs_curve_enum
                                                                              ,config.throw_4_throw_min_distance
                                                                              ,config.throw_4_throw_max_distance
                                                                              ,config.throw_4_throw_strength == 0 ? throw_type_map[ThrowTypeId::none]->throw_strength_ : config.throw_4_throw_strength
                                                                              ,config.throw_4_throw_angle_default == 0 ? throw_type_map[ThrowTypeId::none]->throw_angle_ : ThrowMath::degree_to_radian(config.throw_4_throw_angle, params->pi_)
                                                                              ,config.throw_4_goal_tolerance
                                                                              ,config.throw_4_velocity_adaptation_rate
                                                                              ,config.throw_4_movement_duration == 0 ? throw_type_map[ThrowTypeId::none]->movement_duration_ : config.throw_4_movement_duration
                                                                              ,config.throw_4_movement_share_pick_up == 0 ? throw_type_map[ThrowTypeId::none]->movement_share_pick_up_ : config.throw_4_movement_share_pick_up
                                                                              ,config.throw_4_movement_share_preparation == 0 ? throw_type_map[ThrowTypeId::none]->movement_share_preparation_ : config.throw_4_movement_share_preparation
                                                                              ,config.throw_4_movement_share_throw == 0 ? throw_type_map[ThrowTypeId::none]->movement_share_throw_ : config.throw_4_movement_share_throw
                                                                              ,config.throw_4_movement_share_conclusion == 0 ? throw_type_map[ThrowTypeId::none]->movement_share_conclusion_ : config.throw_4_movement_share_conclusion
                                                                              ,config.throw_4_movement_offset_move_arms_away_from_ball == 0 ? throw_type_map[ThrowTypeId::none]->movement_offset_move_arms_away_from_ball_ : config.throw_4_movement_offset_move_arms_away_from_ball);

            return throw_type_map;
        };
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_TYPE_PARAMETER_BUILDER_H