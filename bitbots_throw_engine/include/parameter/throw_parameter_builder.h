#ifndef BITBOTS_THROW_THROW_PARAMETER_BUILDER_H
#define BITBOTS_THROW_THROW_PARAMETER_BUILDER_H

#include <math.h>
#include <memory>
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

            sp_parameter->start_left_arm_ = request.left_hand_position_;
            sp_parameter->start_right_arm_ = request.right_hand_position_;
            sp_parameter->start_left_feet_ = request.left_feet_position_;
            sp_parameter->start_right_feet_ = request.right_feet_position_;

            sp_parameter->pick_up_left_arm_ = Struct3d{request.ball_position_.x_ - engine_parameter->ball_radius_
                                                      ,request.ball_position_.y_
                                                      ,request.ball_position_.z_};
            sp_parameter->pick_up_right_arm_ = Struct3d{request.ball_position_.x_ + engine_parameter->ball_radius_
                                                       ,request.ball_position_.y_
                                                       ,request.ball_position_.z_};
            sp_parameter->pick_up_trunk_.yaw_ = calculate_angle(request.ball_position_);

            auto throw_start_position_x = - 1.0 * calculate_opposite(throw_orientation_angle, engine_parameter->head_collision_security_radius_ + engine_parameter->ball_radius_);
            auto throw_start_position_y = - 1.0 * calculate_adjacent(throw_orientation_angle, engine_parameter->head_collision_security_radius_ + engine_parameter->ball_radius_);
            sp_parameter->throw_start_left_arm_ = Struct3d{throw_start_position_x
                                                          ,throw_start_position_y
                                                          ,engine_parameter->robot_height_};
            sp_parameter->throw_start_right_arm_ = Struct3d{throw_start_position_x
                                                           ,throw_start_position_y
                                                           ,engine_parameter->robot_height_};
            sp_parameter->throw_start_trunk_.yaw_ = throw_orientation_angle;

            auto throw_zenith_height = engine_parameter->robot_height_ + engine_parameter->head_collision_security_radius_;
            sp_parameter->throw_zenith_left_arm_.z_ = throw_zenith_height;
            sp_parameter->throw_zenith_right_arm_.z_ = throw_zenith_height;

            sp_parameter->throw_velocity_ = calculate_velocity(throw_type->throw_strength_ * engine_parameter->max_throw_velocity_
                                                              ,throw_type->throw_angle_
                                                              ,request.goal_position_
                                                              ,engine_parameter);

            auto throw_release_position_x = calculate_opposite(throw_orientation_angle, engine_parameter->arm_length_ + engine_parameter->ball_radius_);
            auto throw_release_position_y = calculate_adjacent(throw_orientation_angle, engine_parameter->arm_length_ + engine_parameter->ball_radius_);
            auto throw_release_position_z = throw_zenith_height - ((std::cos(throw_type->throw_angle_) / std::sin(throw_type->throw_angle_)) * calculate_distace(Struct3d(throw_release_position_x, throw_release_position_y, 0.0)));

            sp_parameter->throw_release_left_arm_ = Struct3d{throw_release_position_x
                                                            ,throw_release_position_y
                                                            ,throw_release_position_z};
            sp_parameter->throw_release_right_arm_ = Struct3d{throw_release_position_x
                                                             ,throw_release_position_y
                                                             ,throw_release_position_z};
            sp_parameter->throw_release_trunk_.yaw_ = throw_orientation_angle;

            sp_parameter->movement_cycle_frequency_ = engine_parameter->frequency_;

            sp_parameter->pick_up_duration_share_ = throw_type->pick_up_duration_share_;
            sp_parameter->throw_preparation_duration_share_ = throw_type->throw_preparation_duration_share_;
            sp_parameter->throw_duration_share_ = throw_type->throw_duration_share_;
            sp_parameter->throw_conclusion_duration_share_ = throw_type->throw_conclusion_duration_share_;
            check_duration_share(sp_parameter, engine_parameter);

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
        static Struct3d calculate_velocity(double const & velocity
                                          ,double & throw_angle
                                          ,Struct3d & goal_position
                                          ,std::shared_ptr<ThrowEngineParameter> const & engine_parameter){
            double goal_offset;
            double augmented_velocity;
            double const goal_distance = calculate_distace(goal_position);
            bool upper_alert = false;
            bool lower_alert = false;

            while(-30.0 <= throw_angle && 75 >= throw_angle){
                augmented_velocity = velocity;
                goal_offset = fit_throw_velocity(augmented_velocity, throw_angle, goal_distance, engine_parameter);

                // TODO: Rework parameter
                if(0.5 > goal_offset && !lower_alert){
                    throw_angle += 0.5;
                    upper_alert = true;
                }
                else if(0.0 < goal_offset && !upper_alert){
                    throw_angle -= 0.5;
                    lower_alert = true;
                }
                else{
                    break;
                }
            }

            // TODO: check if needed to be reworked
            Struct3d return_velocity{};
            return_velocity.x_ = augmented_velocity * std::cos(throw_angle) * std::sin(calculate_angle(goal_position));
            return_velocity.y_ = augmented_velocity * std::cos(throw_angle) * std::cos(calculate_angle(goal_position));
            return_velocity.z_ = augmented_velocity * std::sin(throw_angle);

            return return_velocity;
        }

        // TODO: Methode describtion
        static double fit_throw_velocity(double & velocity
                                        ,double const & throw_angle
                                        ,double const & goal_distance
                                        ,std::shared_ptr<ThrowEngineParameter> const & engine_parameter){
            double time = 0.0;
            double goal_offset;
            bool upper_alert = false;
            bool lower_alert = false;

            while(engine_parameter->max_throw_velocity_ > velocity && 0.0 < velocity){
                time = calculate_throw_time(engine_parameter->robot_height_, velocity, throw_angle, engine_parameter->gravity_);
                goal_offset = goal_distance - std::abs(velocity * std::cos(throw_angle) * time);

                // TODO: Rework parameter
                if(0.5 > goal_offset && !lower_alert){
                    velocity += 0.5;
                    upper_alert = true;
                }
                else if(0.0 < goal_offset && !upper_alert){
                    velocity -= 0.5;
                    lower_alert = true;
                }
                else{
                    break;
                }
            }

            return goal_offset;
        }

        static double calculate_throw_time(double const & height
                                          ,double const & velocity
                                          ,double const & angle
                                          ,double const & gravity){
            double v_z = velocity * std::sin(angle);
            return (v_z / gravity) * (1 + std::sqrt(1 + ((2 * gravity * height)/v_z)));
        }

        static double calculate_angle(Struct3d & point){
            return atan((point.y_ / point.x_));
        }

        static double fit_angle_for_trigonometric_function(double angle){
            while (angle > 90.0){
                angle -= 90;
            }
            return angle;
        }

        static double calculate_opposite(double angle, double const & hypotenuse){
            return std::sin(fit_angle_for_trigonometric_function(angle)) * hypotenuse;
        }

        static double calculate_adjacent(double angle, double const & hypotenuse){
            return std::cos(fit_angle_for_trigonometric_function(angle)) * hypotenuse;
        }

        static void check_duration_share(std::shared_ptr<ThrowParameter> & sp_parameter
                                        ,std::shared_ptr<ThrowEngineParameter> & engine_parameter){
            double total_share_div = sp_parameter->pick_up_duration_share_
                                     + sp_parameter->throw_preparation_duration_share_
                                     + sp_parameter->throw_duration_share_
                                     + sp_parameter->throw_conclusion_duration_share_
                                     - 1;
            // check if total share is bigger than 1
            if (total_share_div > 0.0000001f){
                // Reduce the shares equality to remain the proportion of the shares 
                sp_parameter->pick_up_duration_share_ -= total_share_div / 4;
                sp_parameter->throw_preparation_duration_share_ -= total_share_div / 4;
                sp_parameter->throw_duration_share_ -= total_share_div / 4;
                sp_parameter->throw_conclusion_duration_share_ -= total_share_div / 4;

                SystemPublisher::publish_warning("The Shares of the throw movement steps are reduced, because the shares added up to a bigger value than 1");
            }
        }
    };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_PARAMETER_BUILDER_H