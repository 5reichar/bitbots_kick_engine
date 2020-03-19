#ifndef THROW_FACTORY_H
#define THROW_FACTORY_H

#include "parameter/throw_type_parameter.h"
#include "throws/throw_curves/throw_curve.h"

class ThrowFactory
{
public:
	std::shared_ptr<ThrowCurve> create_throw(std::shared_ptr<ThrowTypeParameter> & throw_type_parameter, double const & throw_distance);

private:
	ThrowTypeId get_throw_type_id(std::shared_ptr<ThrowTypeParameter> & throw_type_parameter, double const & throw_distance);
	ThrowCurve * create_throw_curve(ThrowTypeId type);
};

#endif