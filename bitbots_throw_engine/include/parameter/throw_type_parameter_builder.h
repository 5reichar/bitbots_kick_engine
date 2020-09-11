#ifndef BITBOTS_THROW_THROW_TYPE_PARAMETER_BUILDER_H
#define BITBOTS_THROW_THROW_TYPE_PARAMETER_BUILDER_H

#include <memory>
#include "parameter/throw_type_parameter.h"
#include <bitbots_throw/throw_paramsConfig.h>

namespace bitbots_throw{
    class ThrowTypeParameterBuilder{
    public:
        static std::shared_ptr<ThrowTypeParameter> build_from_dynamic_reconf(bitbots_throw::throw_paramsConfig& config
                                                                            ,uint32_t level){
            auto sp_parameter = build_default();
            auto current_id = (ThrowTypeId)config.default_throw_enum;
            sp_parameter->default_throw_id_ = current_id;

            current_id = (ThrowTypeId)config.throw_1_throw_enum;
            sp_parameter->map_throw_types_[current_id] = build_throw_type(current_id
                                                                         ,config.throw_1_throw_active
                                                                         ,config.throw_1_throw_priority_level
                                                                         ,config.throw_1_throw_strength
                                                                         ,config.throw_1_throw_angle
                                                                         ,config.throw_1_throw_min_distance
                                                                         ,config.throw_1_throw_max_distance
                                                                         ,config.throw_1_movement_duration
                                                                         ,config.throw_1_movement_share_pick_up
                                                                         ,config.throw_1_movement_share_preparation
                                                                         ,config.throw_1_movement_share_throw
                                                                         ,config.throw_1_movement_share_conclusion);

            current_id = (ThrowTypeId)config.throw_2_throw_enum;
            sp_parameter->map_throw_types_[current_id] = build_throw_type(current_id
                                                                        ,config.throw_2_throw_active
                                                                        ,config.throw_2_throw_priority_level
                                                                        ,config.throw_2_throw_strength
                                                                        ,config.throw_2_throw_angle
                                                                        ,config.throw_2_throw_min_distance
                                                                        ,config.throw_2_throw_max_distance
                                                                        ,config.throw_2_movement_duration
                                                                        ,config.throw_2_movement_share_pick_up
                                                                        ,config.throw_2_movement_share_preparation
                                                                        ,config.throw_2_movement_share_throw
                                                                        ,config.throw_2_movement_share_conclusion);

            current_id = (ThrowTypeId)config.throw_3_throw_enum;
            sp_parameter->map_throw_types_[current_id] = build_throw_type(current_id
                                                                        ,config.throw_3_throw_active
                                                                        ,config.throw_3_throw_priority_level
                                                                        ,config.throw_3_throw_strength
                                                                        ,config.throw_3_throw_angle
                                                                        ,config.throw_3_throw_min_distance
                                                                        ,config.throw_3_throw_max_distance
                                                                        ,config.throw_3_movement_duration
                                                                        ,config.throw_3_movement_share_pick_up
                                                                        ,config.throw_3_movement_share_preparation
                                                                        ,config.throw_3_movement_share_throw
                                                                        ,config.throw_3_movement_share_conclusion);

            current_id = (ThrowTypeId)config.throw_4_throw_enum;
            sp_parameter->map_throw_types_[current_id] = build_throw_type(current_id
                                                                        ,config.throw_4_throw_active
                                                                        ,config.throw_4_throw_priority_level
                                                                        ,config.throw_4_throw_strength
                                                                        ,config.throw_4_throw_angle
                                                                        ,config.throw_4_throw_min_distance
                                                                        ,config.throw_4_throw_max_distance
                                                                        ,config.throw_4_movement_duration
                                                                        ,config.throw_4_movement_share_pick_up
                                                                        ,config.throw_4_movement_share_preparation
                                                                        ,config.throw_4_movement_share_throw
                                                                        ,config.throw_4_movement_share_conclusion);

            sp_parameter->map_throw_types_[ThrowTypeId::none] = build_throw_type((ThrowTypeId)config.default_throw_enum
                                                                          ,true
                                                                          ,0
                                                                          ,config.throw_strength
                                                                          ,config.throw_angle
                                                                          ,0.0
                                                                          ,0.0
                                                                          ,config.movement_duration
                                                                          ,config.movement_share_pick_up
                                                                          ,config.movement_share_preparation
                                                                          ,config.movement_share_throw
                                                                          ,config.movement_share_conclusion);

            return sp_parameter;
        };

        static std::shared_ptr<ThrowTypeParameter> build_default(){
            return std::make_shared<ThrowTypeParameter>(ThrowTypeId::smooth_spline);
        };

        static std::shared_ptr<ThrowType> build_empty_throw_type(){
            return std::make_shared<ThrowType>(ThrowTypeId::testing
                    ,true
                    ,0
                    ,0.0
                    ,0.0
                    ,0.0
                    ,0.0
                    ,0.0
                    ,0.25
                    ,0.25
                    ,0.25
                    ,0.25);
        };

    protected:
        static std::shared_ptr<ThrowType> build_throw_type(ThrowTypeId id
                                                          ,bool active
                                                          ,int priority_level
                                                          ,double min_throw_distance
                                                          ,double max_throw_distance
                                                          ,double throw_strength
                                                          ,double throw_angle
                                                          ,double movement_duration
                                                          ,double movement_share_pick_up
                                                          ,double movement_share_preparation
                                                          ,double movement_share_throw
                                                          ,double movement_share_conclusion){
            return std::make_shared<ThrowType>(id
                                              ,active
                                              ,priority_level
                                              ,min_throw_distance
                                              ,max_throw_distance
                                              ,throw_strength
                                              ,throw_angle
                                              ,movement_duration
                                              ,movement_share_pick_up
                                              ,movement_share_preparation
                                              ,movement_share_throw
                                              ,movement_share_conclusion);
        };
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_TYPE_PARAMETER_BUILDER_H