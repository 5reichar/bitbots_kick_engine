#ifndef BITBOTS_THROW_THROW_FACTORY_H
#define BITBOTS_THROW_THROW_FACTORY_H

#include "parameter/throw_type_parameter.h"
#include "throws/throw_curves/throw_movement.h"
#include "../../bitbots_splines_extension/include/handle/pose_handle.h"

namespace bitbots_throw{
	class ThrowFactory{
	public:
        ThrowTypeId select_throw_type(Struct3d const & throw_goal);
        void set_throw_type_parameter(std::map<ThrowTypeId, std::shared_ptr<ThrowType>> & throw_type_parameter);
        std::shared_ptr<ThrowMovementBase> select_movement(ThrowTypeId throw_type_id);

	private:
        std::shared_ptr<ThrowMaterial> create_material(ThrowCurveId const & left_arm_curve_id, ThrowCurveId const & right_arm_curve_id, ThrowCurveId const & left_leg_curve_id, ThrowCurveId const & right_leg_curve_id);
        template<class c> std::shared_ptr<bitbots_splines::PoseHandle> create_handle();
        std::shared_ptr<bitbots_splines::PoseHandle> select_handle(ThrowCurveId const & throw_curve_id);

        std::map<ThrowTypeId, std::shared_ptr<ThrowType>> throw_type_map_;
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_FACTORY_H