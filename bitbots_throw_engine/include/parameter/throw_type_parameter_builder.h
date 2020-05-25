#ifndef THROW_TYPE_PARAMETER_BUILDER_H
#define THROW_TYPE_PARAMETER_BUILDER_H

#include <memory>
#include "parameter/throw_type_parameter.h"
#include <bitbots_throw_engine/throw_paramsConfig.h>

class ThrowTypeParameterBuilder
{
public:
    static std::shared_ptr<ThrowTypeParameter> build_from_dynamic_reconf(bitbots_throw_engine::throw_paramsConfig& config, uint32_t level)
    {
        auto sp_parameter = build_default();
        auto current_id = (ThrowTypeId)config.default_throw_enum;
        sp_parameter->default_throw_id_ = current_id;

        current_id = (ThrowTypeId)config.beziercurve_throw_enum;
        sp_parameter->map_throw_types_[current_id] = build_throw_type(current_id,
                                                                    config.beziercurve_throw_active,
                                                                    config.beziercurve_throw_priority_level,
                                                                    config.beziercurve_throw_min_distance,
                                                                    config.beziercurve_throw_max_distance,
                                                                    config.beziercurve_pick_up_duration_share,
                                                                    config.beziercurve_throw_preparation_duration_share,
                                                                    config.beziercurve_throw_duration_share,
                                                                    config.beziercurve_throw_conclusion_duration_share,
                                                                    config.beziercurve_throw_angle);

        current_id = (ThrowTypeId)config.linear_spline_throw_enum;
        sp_parameter->map_throw_types_[current_id] = build_throw_type(current_id,
                                                                    config.linear_spline_throw_active,
                                                                    config.linear_spline_throw_priority_level,
                                                                    config.linear_spline_throw_min_distance,
                                                                    config.linear_spline_throw_max_distance,
                                                                    config.linear_spline_pick_up_duration_share,
                                                                    config.linear_spline_throw_preparation_duration_share,
                                                                    config.linear_spline_throw_duration_share,
                                                                    config.linear_spline_throw_conclusion_duration_share,
                                                                    config.linear_spline_throw_angle);

        current_id = (ThrowTypeId)config.cubic_spline_throw_enum;
        sp_parameter->map_throw_types_[current_id] = build_throw_type(current_id,
                                                                    config.cubic_spline_throw_active,
                                                                    config.cubic_spline_throw_priority_level,
                                                                    config.cubic_spline_throw_min_distance,
                                                                    config.cubic_spline_throw_max_distance,
                                                                    config.cubic_spline_pick_up_duration_share,
                                                                    config.cubic_spline_throw_preparation_duration_share,
                                                                    config.cubic_spline_throw_duration_share,
                                                                    config.cubic_spline_throw_conclusion_duration_share,
                                                                    config.cubic_spline_throw_angle);
                                           
        current_id = (ThrowTypeId)config.smooth_spline_throw_enum;                                         
        sp_parameter->map_throw_types_[current_id] = build_throw_type(current_id,
                                                                    config.smooth_spline_throw_active,
                                                                    config.smooth_spline_throw_priority_level,
                                                                    config.smooth_spline_throw_min_distance,
                                                                    config.smooth_spline_throw_max_distance,
                                                                    config.smooth_spline_pick_up_duration_share,
                                                                    config.smooth_spline_throw_preparation_duration_share,
                                                                    config.smooth_spline_throw_duration_share,
                                                                    config.smooth_spline_throw_conclusion_duration_share,
                                                                    config.smooth_spline_throw_angle);

        return sp_parameter;
    };

    static std::shared_ptr<ThrowTypeParameter> build_default()
    {
        return std::make_shared<ThrowTypeParameter>(ThrowTypeId::smooth_spline);
    };

protected:
    static std::shared_ptr<ThrowType> build_throw_type(ThrowTypeId id,
                                                        bool active,
                                                        int priority_level,
                                                        double min_throw_distance,
                                                        double max_throw_distance,
                                                        double pick_up_duration_share,
                                                        double throw_preparation_duration_share,
                                                        double throw_duration_share,
                                                        double throw_conclusion_duration_share,
                                                        double throw_anlge)
    {
        return std::make_shared<ThrowType>(id,
                                            active,
                                            priority_level,
                                            min_throw_distance,
                                            max_throw_distance,
                                            pick_up_duration_share,
                                            throw_preparation_duration_share,
                                            throw_duration_share,
                                            throw_conclusion_duration_share,
                                            throw_anlge);
    };
};

#endif