#ifndef BITBOTS_THROW_THROW_FACTORY_H
#define BITBOTS_THROW_THROW_FACTORY_H

#include "parameter/throw_type_parameter.h"
#include "throws/throw_curves/throw_curve.h"

namespace bitbots_throw{
	class ThrowFactory{
	public:
		std::shared_ptr<ThrowCurve> create_throw(ThrowTypeId throw_type_id);
		ThrowTypeId get_throw_type(std::shared_ptr<ThrowTypeParameter> throw_type_parameter, Struct3d const & throw_goal);

	private:
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_FACTORY_H