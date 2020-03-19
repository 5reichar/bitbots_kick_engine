#ifndef THROW_TYPE_PARAMETER_BUILDER_H
#define THROW_TYPE_PARAMETER_BUILDER_H

#include "parameter/throw_type_parameter.h"
#include <bitbots_throw_engine/throw_paramsConfig.h>

class ThrowTypeParameterBuilder
{
public:
    static ThrowTypeParameter build_from_dynamic_reconf(bitbots_throw_engine::throw_paramsConfig& config, uint32_t level)
    {
        std::shared_ptr<ThrowTypeParameter> sp_parameter;
        sp_parameter->default_throw_id_ = (ThrowTypeId)config.default_throw_enum;

        sp_parameter->v_throw_types_.push_back(ThrowTypeParameterBuilder::build_throw_type((ThrowTypeId)config.beziercurve_throw_enum,
                                                                                    config.beziercurve_throw_active,
                                                                                    config.beziercurve_throw_min_distance,
                                                                                    config.beziercurve_throw_max_distance));
                                                                                    
        sp_parameter->v_throw_types_.push_back(ThrowTypeParameterBuilder::build_throw_type((ThrowTypeId)config.linear_spline_throw_enum,
                                                                                    config.linear_spline_throw_active,
                                                                                    config.linear_spline_throw_min_distance,
                                                                                    config.linear_spline_throw_max_distance));
                                                                                    
        sp_parameter->v_throw_types_.push_back(ThrowTypeParameterBuilder::build_throw_type((ThrowTypeId)config.cubic_spline_throw_enum,
                                                                                    config.cubic_spline_throw_active,
                                                                                    config.cubic_spline_throw_min_distance,
                                                                                    config.cubic_spline_throw_max_distance));
                                                                                    
        sp_parameter->v_throw_types_.push_back(ThrowTypeParameterBuilder::build_throw_type((ThrowTypeId)config.smooth_spline_throw_enum,
                                                                                    config.smooth_spline_throw_active,
                                                                                    config.smooth_spline_throw_min_distance,
                                                                                    config.smooth_spline_throw_max_distance));

        return sp_parameter;
    };

protected:
    static ThrowTypeParameter build_throw_type(ThrowTypeId id, bool active,  double min_throw_distance, double max_throw_distance)
    {
        ThrowTypeParameter type = {id, active,  min_throw_distance, max_throw_distance};
        return type;
    };
};

#endif