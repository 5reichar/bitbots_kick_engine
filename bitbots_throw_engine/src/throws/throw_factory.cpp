#include "throws/throw_factory.h"

#include "throws/throw_curves/beziercurve_throw.h"
#include "throws/throw_curves/linear_spline_throw.h"
#include "throws/throw_curves/cubic_spline_throw.h"
#include "throws/throw_curves/smooth_spline_throw.h"

std::shared_ptr<ThrowCurve> ThrowFactory::create_throw(std::shared_ptr<ThrowType> throw_type)
{
	//TODO: testing
	//TODO: cleanup
	std::shared_ptr<ThrowCurve> sp_return;
	ThrowCurve * p_return_throw = nullptr;

	switch (throw_type->id_)
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

	sp_return.reset(p_return_throw);
	return sp_return;
}

std::shared_ptr<ThrowType> ThrowFactory::get_throw_type(std::shared_ptr<ThrowTypeParameter> throw_type_parameter, double const & throw_distance)
{
	std::shared_ptr<ThrowType> sp_throw_type = std::make_shared<ThrowType>(throw_type_parameter->default_throw_);

	for (auto it = throw_type_parameter->v_throw_types_.begin(); it != throw_type_parameter->v_throw_types_.end(); ++it)
	{
		if (it->active_ &&
			it->min_throw_distance_ < throw_distance &&
			it->max_throw_distance_ > throw_distance)
		{
			sp_throw_type = std::make_shared<ThrowType>(*it);
			break;
		}
	}

	return sp_throw_type;
}