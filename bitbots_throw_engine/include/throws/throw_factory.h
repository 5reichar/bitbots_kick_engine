#ifndef BITBOTS_THROW_THROW_FACTORY_H
#define BITBOTS_THROW_THROW_FACTORY_H

#include "parameter/throw_type_parameter.h"
#include "throws/throw_curves/throw_movement_base.h"
#include "../../bitbots_splines_extension/include/handle/pose_handle.h"

namespace bitbots_throw{
	class ThrowFactory{
	public:
        ThrowTypeId select_throw_type(Struct3d const & throw_goal);
        void set_engine_parameter(std::shared_ptr<RobotAndWorldParameter> & parameter);
        void set_throw_type_parameter(std::map<ThrowTypeId, std::shared_ptr<ThrowType>> & throw_type_parameter);
        std::shared_ptr<ThrowMovementBase> select_movement(ThrowTypeId throw_type_id, const ThrowRequest & request);

	private:
        std::shared_ptr<ThrowMovementBase> create_movement(std::shared_ptr<ThrowMaterial> & material, std::shared_ptr<ThrowType> & type, const ThrowRequest & request);
        std::shared_ptr<ThrowMovementBase> create_movement_position_only(std::shared_ptr<ThrowMaterial> & material, std::shared_ptr<ThrowType> & type, const ThrowRequest & request);
        std::shared_ptr<ThrowMovementBase> create_movement_testing(std::shared_ptr<ThrowMaterial> & material, std::shared_ptr<ThrowType> & type, const ThrowRequest & request);

        std::shared_ptr<ThrowMaterial> create_material(ThrowCurveId const & left_arm_curve_id, ThrowCurveId const & right_arm_curve_id, ThrowCurveId const & left_leg_curve_id, ThrowCurveId const & right_leg_curve_id);
        template<class c> std::shared_ptr<bitbots_splines::PoseHandle> create_handle();
        std::shared_ptr<bitbots_splines::PoseHandle> select_handle(ThrowCurveId const & throw_curve_id);

        std::map<ThrowTypeId, std::shared_ptr<ThrowType>> throw_type_map_;
        std::shared_ptr<RobotAndWorldParameter> sp_engine_parameter_;
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_FACTORY_H