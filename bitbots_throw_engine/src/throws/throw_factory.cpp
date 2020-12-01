#include "throws/throw_factory.h"

#include "utility/throw_math.h"
#include "ros_interface/publisher/system_publisher.h"
#include "../../bitbots_splines_extension/include/spline/beziercurve.h"
#include "../../bitbots_splines_extension/include/spline/linear_spline.h"
#include "../../bitbots_splines_extension/include/spline/cubic_spline.h"
#include "../../bitbots_splines_extension/include/spline/quintic_spline.h"
#include "throws/throw_curves/throw_movement.h"
#include "throws/throw_curves/throw_movement_position_only.h"
#include "throws/throw_curves/testing_movement.h"

namespace bitbots_throw{
    ThrowTypeId ThrowFactory::select_throw_type(Struct3d const & throw_goal){
        ThrowTypeId throw_type_id = ThrowTypeId::none;
        auto throw_distance = ThrowMath::calculate_distance(throw_goal);

        for (auto & map_throw_type : throw_type_map_){
            auto throw_type = map_throw_type.second;
            if (throw_type->active_ &&
                throw_type->min_throw_distance_ < throw_distance &&
                throw_type->max_throw_distance_ >= throw_distance){
                throw_type_id = map_throw_type.first;
                break;
            }
        }

        return throw_type_id;
    }

    void ThrowFactory::set_engine_parameter(std::shared_ptr<RobotAndWorldParameter> & parameter){
        sp_engine_parameter_ = parameter;
    }

    void ThrowFactory::set_throw_type_parameter(std::map<ThrowTypeId, std::shared_ptr<ThrowType>> & throw_type_parameter){
        throw_type_map_ = throw_type_parameter;
    }

	std::shared_ptr<ThrowMovementBase> ThrowFactory::select_movement(ThrowTypeId throw_type_id, const ThrowRequest & request){
		std::shared_ptr<ThrowMovementBase> sp_return;
        std::shared_ptr<ThrowType> type;

        if (throw_type_map_.cend() == throw_type_map_.find(throw_type_id)){
            type = throw_type_map_[ThrowTypeId::none];
        }else{
            type = throw_type_map_[throw_type_id];
        }
		auto movement_id = throw_type_map_[throw_type_id]->movement_;
        auto arms_id = throw_type_map_[throw_type_id]->arms_curve_;
        auto legs_id = throw_type_map_[throw_type_id]->legs_curve_;
        auto material = create_material(arms_id, arms_id, legs_id, legs_id);

		switch (movement_id){
            case ThrowMovementId::throw_movement:
                sp_return = create_movement(material, type, request);
                break;
            case ThrowMovementId::throw_movement_position_only:
                sp_return = create_movement_position_only(material, type, request);
                break;
            case ThrowMovementId::testing:
                sp_return = create_movement_testing(material, type, request);
                break;
        }

		return sp_return;
	}

    std::shared_ptr<ThrowMovementBase> ThrowFactory::create_movement(std::shared_ptr<ThrowMaterial> & material, std::shared_ptr<ThrowType> & type, const ThrowRequest & request){
        auto movement = std::make_shared<ThrowMovement>(material);
        movement->init(std::make_shared<ThrowService>(request, *type, *sp_engine_parameter_));
        return movement;
    }

    std::shared_ptr<ThrowMovementBase> ThrowFactory::create_movement_position_only(std::shared_ptr<ThrowMaterial> & material, std::shared_ptr<ThrowType> & type, const ThrowRequest & request){
        auto movement = std::make_shared<ThrowMovementPositionOnly>(material);
        movement->init(std::make_shared<ThrowService>(request, *type, *sp_engine_parameter_));
        return movement;
    }

    std::shared_ptr<ThrowMovementBase> ThrowFactory::create_movement_testing(std::shared_ptr<ThrowMaterial> & material, std::shared_ptr<ThrowType> & type, const ThrowRequest & request){
        auto movement = std::make_shared<TestingMovement>(material);
        movement->init(std::make_shared<ThrowService>(request, *type, *sp_engine_parameter_));
        return movement;
    }

    std::shared_ptr<ThrowMaterial> ThrowFactory::create_material(ThrowCurveId const & left_arm_curve_id
                                                                ,ThrowCurveId const & right_arm_curve_id
                                                                ,ThrowCurveId const & left_leg_curve_id
                                                                ,ThrowCurveId const & right_leg_curve_id){
        std::shared_ptr<ThrowMaterial> material = std::make_shared<ThrowMaterial>(
                select_handle(left_arm_curve_id),
                select_handle(right_arm_curve_id),
                select_handle(left_leg_curve_id),
                select_handle(right_leg_curve_id));

        return material;
    }

    template<class c>
    std::shared_ptr<bitbots_splines::PoseHandle> ThrowFactory::create_handle(){
        return std::make_shared<bitbots_splines::PoseHandle>(
                std::make_shared<c>() // x
                ,std::make_shared<c>() // y
                ,std::make_shared<c>() // z
                ,std::make_shared<c>() // roll
                ,std::make_shared<c>() // pitch
                ,std::make_shared<c>()); // yaw
    }

    std::shared_ptr<bitbots_splines::PoseHandle> ThrowFactory::select_handle(ThrowCurveId const & throw_curve_id){
        std::shared_ptr<bitbots_splines::PoseHandle> return_value;

        switch (throw_curve_id){
            case ThrowCurveId::beziercurve:
                return_value = create_handle<bitbots_splines::Beziercurve>();
                break;
            case ThrowCurveId::linear_spline:
                return_value = create_handle<bitbots_splines::LinearSpline>();
                break;
            case ThrowCurveId::cubic_spline:
                return_value = create_handle<bitbots_splines::CubicSpline>();
                break;
            case ThrowCurveId::smooth_spline:
                return_value = create_handle<bitbots_splines::SmoothSpline>();
                break;
            default:
                return_value = create_handle<bitbots_splines::CubicSpline>();
                break;
        }

        return return_value;
    }
} //bitbots_throw