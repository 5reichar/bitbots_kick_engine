#include "throws/throw_factory.h"

#include "utility/throw_math.h"
#include "ros_interface/publisher/system_publisher.h"
#include "../../bitbots_splines_extension/include/spline/beziercurve.h"
#include "../../bitbots_splines_extension/include/spline/linear_spline.h"
#include "../../bitbots_splines_extension/include/spline/cubic_spline.h"
#include "../../bitbots_splines_extension/include/spline/smooth_spline.h"
#include "throws/throw_curves/testing_movement.h"

namespace bitbots_throw{
    ThrowTypeId ThrowFactory::get_throw_type(Struct3d const & throw_goal){
        ThrowTypeId throw_type_id = ThrowTypeId::none;
        auto throw_distance = ThrowMath::calculate_distance(throw_goal);

        for (auto & map_throw_type : sp_throw_type_parameter_->map_throw_types_){
            auto throw_type = map_throw_type.second;
            if (throw_type->active_ &&
                throw_type->min_throw_distance_ < throw_distance &&
                throw_type->max_throw_distance_ > throw_distance){
                throw_type_id = map_throw_type.first;
                break;
            }
        }

        return throw_type_id;
    }

    void ThrowFactory::set_throw_type_parameter(std::shared_ptr<ThrowTypeParameter> & throw_type_parameter){
        sp_throw_type_parameter_ = throw_type_parameter;
    }

	std::shared_ptr<ThrowMovement> ThrowFactory::select_movement(ThrowTypeId throw_type_id){
		std::shared_ptr<ThrowMovement> sp_return;

        if(ThrowTypeId::none == throw_type_id){
            throw_type_id = sp_throw_type_parameter_->default_throw_id_;
        }

		switch (throw_type_id){
		case ThrowTypeId::beziercurve:
			sp_return = std::make_shared<ThrowMovement>(create_curve<bitbots_splines::Beziercurve>());
			SystemPublisher::publish_info("BeziercurveThrow Selected");
			break;
		case ThrowTypeId::linear_spline:
            sp_return = std::make_shared<ThrowMovement>(create_curve<bitbots_splines::LinearSpline>());
            SystemPublisher::publish_info("LinearSplineThrow Selected");
			break;
		case ThrowTypeId::cubic_spline:
            sp_return = std::make_shared<ThrowMovement>(create_curve<bitbots_splines::CubicSpline>());
            SystemPublisher::publish_info("CubicSplineThrow Selected");
			break;
		case ThrowTypeId::smooth_spline:
            sp_return = std::make_shared<ThrowMovement>(create_curve<bitbots_splines::SmoothSpline>());
            SystemPublisher::publish_info("SmoothSplineThrow Selected");
            break;
        case ThrowTypeId::testing:
        default:
            sp_return = std::make_shared<TestingMovement>(create_curve<bitbots_splines::CubicSpline>());
            SystemPublisher::publish_info("Testing Selected");
            break;
        }

		return sp_return;
	}

    template<class c>
    std::shared_ptr<ThrowMaterial> ThrowFactory::create_curve(){
        std::shared_ptr<ThrowMaterial> material = std::make_shared<ThrowMaterial>(
                create_handle<c>(), // Left Hand
                create_handle<c>(), // Right Hand
                create_handle<c>(), // Left Feet
                create_handle<c>());// Right Feet

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

} //bitbots_throw