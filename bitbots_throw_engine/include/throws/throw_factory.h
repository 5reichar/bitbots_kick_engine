#ifndef BITBOTS_THROW_THROW_FACTORY_H
#define BITBOTS_THROW_THROW_FACTORY_H

#include "parameter/throw_type_parameter.h"
#include "throws/throw_curves/throw_movement.h"
#include "../../bitbots_splines_extension/include/handle/pose_handle.h"

namespace bitbots_throw{
	class ThrowFactory{
	public:
        ThrowTypeId get_throw_type(Struct3d const & throw_goal);
        void set_throw_type_parameter(std::shared_ptr<ThrowTypeParameter> & throw_type_parameter);

        std::shared_ptr<ThrowMovement> create_movement(ThrowTypeId throw_type_id);

	private:
        template<class c> std::shared_ptr<ThrowMaterial> create_curve();
        template<class c> std::shared_ptr<bitbots_splines::PoseHandle> create_handle();

        std::shared_ptr<ThrowTypeParameter> sp_throw_type_parameter_;
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_FACTORY_H