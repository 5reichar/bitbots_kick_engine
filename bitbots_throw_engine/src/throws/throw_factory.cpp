#include "throws/throw_factory.h"

#include "throws/throw_curves/beziercurve_throw.h"
#include "throws/throw_curves/linear_spline_throw.h"
#include "throws/throw_curves/cubic_spline_throw.h"
#include "throws/throw_curves/smooth_spline_throw.h"
#include "utility/throw_utilities.h"

namespace bitbots_throw{
	std::shared_ptr<ThrowCurve> ThrowFactory::create_throw(ThrowTypeId throw_type_id){
		std::shared_ptr<ThrowCurve> sp_return;

		switch (throw_type_id){
		case ThrowTypeId::beziercurve:
			sp_return.reset(new BeziercurveThrow());
			break;
		case ThrowTypeId::linear_spline:
			sp_return.reset(new LinearSplineThrow());
			break;
		case ThrowTypeId::cubic_spline:
			sp_return.reset(new CubicSplineThrow());
			break;
		case ThrowTypeId::smooth_spline:
			sp_return.reset(new SmoothSplineThrow());
			break;
		default:
			sp_return.reset(new SmoothSplineThrow());
			break;
		}

		return sp_return;
	}

	ThrowTypeId ThrowFactory::get_throw_type(std::shared_ptr<ThrowTypeParameter> throw_type_parameter, Struct3d const & throw_goal){
		ThrowTypeId throw_type_id = throw_type_parameter->default_throw_id_;
		auto throw_distance = calculate_distace(throw_goal);

		for (auto it = throw_type_parameter->map_throw_types_.begin(); it != throw_type_parameter->map_throw_types_.end(); ++it){
			auto throw_type = it->second;
			if (throw_type->active_ &&
				throw_type->min_throw_distance_ < throw_distance &&
				throw_type->max_throw_distance_ > throw_distance){
				throw_type_id = it->first;
				break;
			}
		}

		return throw_type_id;
	}
} //bitbots_throw