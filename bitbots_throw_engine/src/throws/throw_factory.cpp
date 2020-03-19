#include "throws/throw_factory.h"

#include "throws/throw_curves/beziercurve_throw.h"
#include "throws/throw_curves/linear_spline_throw.h"
#include "throws/throw_curves/cubic_spline_throw.h"
#include "throws/throw_curves/smooth_spline_throw.h"

std::shared_ptr<ThrowCurve> ThrowFactory::create_throw(std::shared_ptr<ThrowTypeParameter> & throw_type_parameter, double const & throw_distance)
{
	//TODO: testing
	//TODO: cleanup
	std::shared_ptr<ThrowCurve> sp_return;

	sp_return.reset(create_throw_curve(get_throw_type_id(throw_type_parameter, throw_distance)));

	return sp_return;
}

ThrowTypeId ThrowFactory::get_throw_type_id(std::shared_ptr<ThrowTypeParameter> & throw_type_parameter, double const & throw_distance)
{
	ThrowTypeId enum_throw_type = throw_type_parameter->default_throw_id_;

	for (auto it = throw_type_parameter->v_throw_types_.begin(); it != throw_type_parameter->v_throw_types_.end(); ++it)
	{
		if (it->active_ &&
			it->min_throw_distance_ < throw_distance &&
			it->max_throw_distance_ > throw_distance)
		{
			enum_throw_type = it->id_;
			break;
		}
	}

	return enum_throw_type;
}

ThrowCurve * ThrowFactory::create_throw_curve(ThrowTypeId type)
{
	ThrowCurve * p_return_throw = nullptr;

	switch (type)
	{
	case ThrowTypeId::beziercurve:
		p_return_throw = new BeziercurveThrow();
		break;
	case ThrowTypeId::linear_spline:
		p_return_throw = new LinearSplineThrow();
		break;
	case ThrowTypeId::cubic_spline:
		p_return_throw = new CubicSplineThrow();
		break;
	case ThrowTypeId::smooth_spline:
		p_return_throw = new SmoothSplineThrow();
		break;
	default:
		p_return_throw = new SmoothSplineThrow();
		break;
	}

	return p_return_throw;
}