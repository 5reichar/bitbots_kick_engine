#ifndef BITBOTS_THROW_THROW_PARAMETER_BUILDER_H
#define BITBOTS_THROW_THROW_PARAMETER_BUILDER_H

#include <math.h>
#include <memory>
#include "utility/throw_math.h"
#include "utility/throw_utilities.h"
#include "parameter/throw_parameter.h"
#include "parameter/throw_type_parameter.h"
#include "parameter/throw_engine_parameter.h"
#include "ros_interface/publisher/system_publisher.h"

namespace bitbots_throw{
    class ThrowParameterBuilder{
    public:
        static std::shared_ptr<ThrowParameter> build_from_dynamic_reconf(std::shared_ptr<ThrowEngineParameter> & engine_parameter
                                                                        ,std::shared_ptr<ThrowType> & throw_type
                                                                        ,ThrowRequest request){
            auto sp_parameter = build_default();
            auto throw_orientation_angle = calculate_angle(request.goal_position_);

            sp_parameter->movement_duration_ = throw_type->movement_duration_;
            sp_parameter->movement_share_pick_up_ = throw_type->movement_share_pick_up_;
            sp_parameter->movement_share_preparation_ = throw_type->movement_share_preparation_;
            sp_parameter->movement_share_throw_ = throw_type->movement_share_throw_;
            sp_parameter->movement_share_conclusion_ = throw_type->movement_share_conclusion_;
            check_duration_share(sp_parameter, engine_parameter);

            sp_parameter->start_left_arm_ = request.left_hand_position_;
            sp_parameter->start_right_arm_ = request.right_hand_position_;
            sp_parameter->start_left_feet_ = request.left_feet_position_;
            sp_parameter->start_right_feet_ = request.right_feet_position_;

            sp_parameter->pick_up_left_arm_ = request.ball_position_;
            sp_parameter->pick_up_left_arm_.y_ += engine_parameter->ball_radius_;
            sp_parameter->pick_up_right_arm_ = request.ball_position_;
            sp_parameter->pick_up_right_arm_.y_ -= engine_parameter->ball_radius_;
            sp_parameter->pick_up_trunk_.yaw_ = calculate_angle(request.ball_position_);

            sp_parameter->throw_start_left_arm_.x_ = -0.5 * engine_parameter->arm_length_;
            sp_parameter->throw_start_left_arm_.y_ = sp_parameter->pick_up_left_arm_.y_;
            sp_parameter->throw_start_left_arm_.z_ = engine_parameter->trunk_height_ + engine_parameter->head_height_/2;

            sp_parameter->throw_start_right_arm_ = sp_parameter->throw_start_left_arm_;
            sp_parameter->throw_start_right_arm_.y_ = sp_parameter->pick_up_right_arm_.y_;

            sp_parameter->throw_start_trunk_.yaw_ = throw_orientation_angle;

            auto throw_zenith_height = engine_parameter->trunk_height_ + engine_parameter->arm_length_;
            sp_parameter->throw_zenith_left_arm_.z_ = throw_zenith_height;
            sp_parameter->throw_zenith_left_arm_.y_ = sp_parameter->pick_up_left_arm_.y_;
            sp_parameter->throw_zenith_right_arm_.z_ = throw_zenith_height;
            sp_parameter->throw_zenith_right_arm_.y_ = sp_parameter->pick_up_right_arm_.y_;

            auto throw_release_position = calculate_throw_release_point(throw_type->throw_angle_
                                                                       ,engine_parameter->arm_length_
                                                                       ,throw_orientation_angle
                                                                       ,throw_zenith_height);
            sp_parameter->throw_release_left_arm_ = throw_release_position;
            sp_parameter->throw_release_right_arm_ = throw_release_position;
            sp_parameter->throw_release_right_arm_.y_ = -1 * throw_release_position.y_;
            sp_parameter->throw_release_trunk_.yaw_ = throw_orientation_angle;

            sp_parameter->throw_velocity_ = calculate_velocity(request.goal_position_, sp_parameter, engine_parameter);

            return sp_parameter;
        };

        static std::shared_ptr<ThrowParameter> build_default(){
            // TODO: enter better default values
            return std::make_shared<ThrowParameter>(Struct3dRPY{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  //start_left_hand
                                                   ,Struct3dRPY{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  //start_right_hand
                                                   ,Struct3dRPY{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  //start_left_feet
                                                   ,Struct3dRPY{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  //start_right_feet
                                                   ,Struct3dRPY{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  //pick_up_left_hand
                                                   ,Struct3dRPY{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  //pick_up_right_hand
                                                   ,Struct3dRPY{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  //pick_up_trunk
                                                   ,Struct3dRPY{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  //throw_start_left_hand
                                                   ,Struct3dRPY{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  //throw_start_right_hand
                                                   ,Struct3dRPY{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  //throw_start_trunk
                                                   ,Struct3dRPY{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  //throw_zenith_left_hand
                                                   ,Struct3dRPY{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  //throw_zenith_right_hand
                                                   ,Struct3dRPY{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  //throw_release_left_hand
                                                   ,Struct3dRPY{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  //throw_release_right_hand
                                                   ,Struct3dRPY{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  //throw_release_trunk
                                                   ,Struct3d{0.0, 0.0, 0.0}  //throw_velocity
                                                   ,0.0  //movement_cycle_frequency
                                                   ,0.25  //pick_up_duration_share
                                                   ,0.25  //throw_preparation_duration_share
                                                   ,0.25  //throw_duration_share
                                                   ,0.25);  //throw_conclusion_duration_share
        };

    protected:
        static Struct3d calculate_throw_release_point(double const & throw_release_angle
                                                     ,double const & arm_length
                                                     ,double const & throw_orientation_angle
                                                     ,double const & throw_zenith){
            ThrowMath throw_math;
            Struct3d throw_release_point;
            // From Isosceles triangle with the two angles with the same size are given as 90 - throw_angle
            auto hypotenuse = std::sqrt(2 * std::pow(arm_length, 2) * (1 - throw_math.cos_deg(2 * throw_release_angle)));
            auto opposite = throw_math.sin_deg(throw_release_angle) * hypotenuse;
            auto adjacent = throw_math.cos_deg(throw_release_angle) * hypotenuse;

            throw_release_point.x_ = throw_math.cos_deg(throw_orientation_angle) * adjacent;
            throw_release_point.y_ = throw_math.sin_deg(throw_orientation_angle) * adjacent;
            throw_release_point.z_ = throw_zenith - opposite;

            // Debug Stuff
            std::stringstream stream;
            stream << "ThrowAngle: " << throw_release_angle << "; ArmLength: " << arm_length << "; ThrowZenith: " << throw_zenith;
            stream << "; Hypot: " << hypotenuse << "; oppos: " << opposite << "; adj: " << adjacent;
            stream << "; Point.z_: " << throw_release_point.z_ << "; sin(tra): " << throw_math.sin_deg(throw_release_angle);
            SystemPublisher::publish_info(stream.str(), "Utility::calculate_throw_release_point");

            return throw_release_point;
        };

        static Struct3d calculate_velocity(Struct3d & goal_position
                                          ,std::shared_ptr<ThrowParameter> const & throw_parameter
                                          ,std::shared_ptr<ThrowEngineParameter> const & engine_parameter){
            ThrowMath throw_math;

            double fly_time = std::sqrt((throw_parameter->throw_release_right_arm_.z_ + engine_parameter->trunk_height_/2) / engine_parameter->gravity_);
            double max_torque = engine_parameter->arm_max_stall_torque_ * engine_parameter->arm_stall_torque_usage_;
            double throw_release_velocity = throw_math.calculate_distance(goal_position) / fly_time;
            double needed_torque = (throw_release_velocity * engine_parameter->ball_weight_ * engine_parameter->arm_length_) / (throw_parameter->movement_duration_ * throw_parameter->movement_share_throw_);

            if(max_torque < (needed_torque/2)){
                // TODO: Error handling
            }

            // TODO: check if needed to be reworked
            Struct3d return_velocity{};
            return_velocity.x_ = throw_release_velocity * throw_math.cos_deg(calculate_angle(goal_position));
            return_velocity.y_ = throw_release_velocity * throw_math.sin_deg(calculate_angle(goal_position));

            return return_velocity;
        }

        static double calculate_angle(Struct3d & point){
            return atan((point.y_ / point.x_));
        }

        static void check_duration_share(std::shared_ptr<ThrowParameter> & sp_parameter
                                        ,std::shared_ptr<ThrowEngineParameter> & engine_parameter){
            double total_share_div = sp_parameter->movement_share_pick_up_
                                     + sp_parameter->movement_share_preparation_
                                     + sp_parameter->movement_share_throw_
                                     + sp_parameter->movement_share_conclusion_
                                     - 1;
            // check if total share is bigger than 1
            if (total_share_div > 0.0000001f){
                // Reduce the shares equality to remain the proportion of the shares 
                sp_parameter->movement_share_pick_up_ -= total_share_div / 4;
                sp_parameter->movement_share_preparation_ -= total_share_div / 4;
                sp_parameter->movement_share_throw_ -= total_share_div / 4;
                sp_parameter->movement_share_conclusion_ -= total_share_div / 4;

                SystemPublisher::publish_warning("The Shares of the throw movement steps are reduced, because the shares added up to a bigger value than 1");
            }
        }
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_PARAMETER_BUILDER_H