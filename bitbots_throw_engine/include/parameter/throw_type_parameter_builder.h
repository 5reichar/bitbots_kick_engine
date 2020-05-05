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
        std::shared_ptr<ThrowTypeParameter> sp_parameter;
        sp_parameter->default_throw_ = build_throw_type((ThrowTypeId)config.default_throw_enum, true, 10, 0, 0);

        sp_parameter->v_throw_types_.push_back(ThrowTypeParameterBuilder::build_throw_type((ThrowTypeId)config.beziercurve_throw_enum,
                                                                                    config.beziercurve_throw_active,
                                                                                    config.beziercurve_throw_priority_level,
                                                                                    config.beziercurve_throw_min_distance,
                                                                                    config.beziercurve_throw_max_distance,
                                                                                    config.beziercurve_pick_up_duration_share,
                                                                                    config.beziercurve_throw_preparation_duration_share,
                                                                                    config.beziercurve_throw_duration_share,
                                                                                    config.beziercurve_throw_angle));
                                                                                    
        sp_parameter->v_throw_types_.push_back(ThrowTypeParameterBuilder::build_throw_type((ThrowTypeId)config.linear_spline_throw_enum,
                                                                                    config.linear_spline_throw_active,
                                                                                    config.linear_spline_throw_priority_level,
                                                                                    config.linear_spline_throw_min_distance,
                                                                                    config.linear_spline_throw_max_distance,
                                                                                    config.linear_spline_pick_up_duration_share,
                                                                                    config.linear_spline_throw_preparation_duration_share,
                                                                                    config.linear_spline_throw_duration_share,
                                                                                    config.linear_spline_throw_angle));
                                                                                    
        sp_parameter->v_throw_types_.push_back(ThrowTypeParameterBuilder::build_throw_type((ThrowTypeId)config.cubic_spline_throw_enum,
                                                                                    config.cubic_spline_throw_active,
                                                                                    config.cubic_spline_throw_priority_level,
                                                                                    config.cubic_spline_throw_min_distance,
                                                                                    config.cubic_spline_throw_max_distance,
                                                                                    config.cubic_spline_pick_up_duration_share,
                                                                                    config.cubic_spline_throw_preparation_duration_share,
                                                                                    config.cubic_spline_throw_duration_share,
                                                                                    config.cubic_spline_throw_angle));
                                                                                    
        sp_parameter->v_throw_types_.push_back(ThrowTypeParameterBuilder::build_throw_type((ThrowTypeId)config.smooth_spline_throw_enum,
                                                                                    config.smooth_spline_throw_active,
                                                                                    config.smooth_spline_throw_priority_level,
                                                                                    config.smooth_spline_throw_min_distance,
                                                                                    config.smooth_spline_throw_max_distance,
                                                                                    config.smooth_spline_pick_up_duration_share,
                                                                                    config.smooth_spline_throw_preparation_duration_share,
                                                                                    config.smooth_spline_throw_duration_share,
                                                                                    config.smooth_spline_throw_angle));

        return sp_parameter;
    };

protected:
    static ThrowType build_throw_type(ThrowTypeId id, bool active, int priority_level,  double min_throw_distance, double max_throw_distance, double pick_up_duration_share, double throw_preparation_duration_share, double throw_duration_share, double throw_anlge)
    {
        ThrowType type = {id, active, priority_level, min_throw_distance, max_throw_distance, pick_up_duration_share, throw_preparation_duration_share, throw_duration_share, throw_anlge};
        return type;
    };
};

#endif