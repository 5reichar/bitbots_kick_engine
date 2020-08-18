#ifndef BITBOTS_THROW_THROW_FACTORY_H
#define BITBOTS_THROW_THROW_FACTORY_H

#include "parameter/throw_type_parameter.h"
#include "throws/throw_curves/throw_curve.h"

namespace bitbots_throw{
	class ThrowFactory{
	public:
        ThrowTypeId get_throw_type(Struct3d const & throw_goal);
        std::shared_ptr<ThrowCurve> create_throw(ThrowTypeId throw_type_id);
        void set_throw_type_parameter(std::shared_ptr<ThrowTypeParameter> & throw_type_parameter);

	private:
        std::shared_ptr<ThrowTypeParameter> sp_throw_type_parameter_;
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_FACTORY_H