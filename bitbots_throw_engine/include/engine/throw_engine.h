#ifndef THROW_ENGINE_H
#define THROW_ENGINE_H

#include "throws/throw_factory.h"
#include "parameter/throw_parameter_builder.h"
#include "parameter/throw_type_parameter.h"
#include "parameter/throw_engine_parameter.h"

class ThrowEngine
{
public:
    void set_throw_types(std::shared_ptr<ThrowTypeParameter> types);
    void set_engine_parameter(std::shared_ptr<ThrowEngineParameter> parameter);

    std::shared_ptr<ThrowCurve> get_current_throw() const;

	virtual bool throw_ball(Struct3d & ball_position, Struct3d & goal_position);

private:

    std::shared_ptr<ThrowCurve> sp_current_throw_;
    std::shared_ptr<ThrowFactory> sp_throw_factory_;
    std::shared_ptr<ThrowTypeParameter> sp_throw_types_;
    std::shared_ptr<ThrowEngineParameter> sp_engine_parameter_;
};

#endif