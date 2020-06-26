#include "throws/throw_curves/throw_curve.h"

namespace bitbots_throw{
    ThrowCurve::ThrowCurve(std::shared_ptr<bitbots_splines::PoseHandle> left_hand,
                           std::shared_ptr<bitbots_splines::PoseHandle> right_hand,
                           std::shared_ptr<bitbots_splines::PoseHandle> trunk,
                           std::shared_ptr<bitbots_splines::PoseHandle> left_feet,
                           std::shared_ptr<bitbots_splines::PoseHandle> right_feet)
            : sp_pose_left_hand_(std::move(left_hand))
            , sp_pose_right_hand_(std::move(right_hand))
            , sp_pose_trunk_(std::move(trunk))
            , sp_pose_left_feet_(std::move(left_feet))
            , sp_pose_right_feet_(std::move(right_feet)){
    }

    double ThrowCurve::calculate_trajectories(std::shared_ptr<ThrowParameter> & throw_parameter){

        double movement_time = 0.0;

        //checks
        if(check_requirements(throw_parameter)){
            calculate_pick_up_ball_movement(movement_time, throw_parameter);
            calculate_throw_preparation_movement(movement_time, throw_parameter);
            calculate_throw_movement(movement_time, throw_parameter);
            calculate_throw_conclusion_movement(movement_time, throw_parameter);
        }
        //TODO: implement ROS Error

        return movement_time;
    }

    bool ThrowCurve::check_requirements(std::shared_ptr<ThrowParameter> & throw_parameter){
        return true;
    }

    std::shared_ptr<bitbots_splines::PoseHandle> ThrowCurve::get_pose_left_hand() const{
        return sp_pose_left_hand_;
    }

    std::shared_ptr<bitbots_splines::PoseHandle> ThrowCurve::get_pose_right_hand() const{
        return sp_pose_right_hand_;
    }

    std::shared_ptr<bitbots_splines::PoseHandle> ThrowCurve::get_pose_trunk() const{
        return sp_pose_trunk_;
    }

    std::shared_ptr<bitbots_splines::PoseHandle> ThrowCurve::get_pose_left_feet() const{
        return sp_pose_left_feet_;
    }

    std::shared_ptr<bitbots_splines::PoseHandle> ThrowCurve::get_pose_right_feet() const{
        return sp_pose_right_feet_;
    }

    void ThrowCurve::calculate_pick_up_ball_movement(double & time, std::shared_ptr<ThrowParameter> & throw_parameter){
        /////  Preparation
        //Set up the trajectories for the half cycle (single step)
        double start_time = time;
        double pick_up_ball_time =
                time + 1 / (throw_parameter->pick_up_duration_share_ * throw_parameter->movement_cycle_frequency_);

        ////  Movement
        add_points(sp_pose_left_hand_, start_time, throw_parameter->start_left_arm_);
        add_points(sp_pose_left_hand_, pick_up_ball_time, throw_parameter->pick_up_left_arm_);
        add_points(sp_pose_right_hand_, start_time, throw_parameter->start_right_arm_);
        add_points(sp_pose_right_hand_, pick_up_ball_time, throw_parameter->pick_up_right_arm_);
        add_points(sp_pose_trunk_, start_time, Struct3dRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        add_points(sp_pose_trunk_, pick_up_ball_time, throw_parameter->pick_up_trunk_);
        add_points(sp_pose_left_feet_, start_time, throw_parameter->start_left_feet_);
        add_points(sp_pose_left_feet_, pick_up_ball_time, Struct3dRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        add_points(sp_pose_right_feet_, start_time, throw_parameter->start_right_feet_);
        add_points(sp_pose_right_feet_, pick_up_ball_time, Struct3dRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        /////  Clean Up
        time = pick_up_ball_time;
    }

    void
    ThrowCurve::calculate_throw_preparation_movement(double & time, std::shared_ptr<ThrowParameter> & throw_parameter){
        /////  Preparation
        //Set up the trajectories for the half cycle (single step)
        double begin_throw_time = time + 1 / (throw_parameter->throw_preparation_duration_share_ *
                                              throw_parameter->movement_cycle_frequency_);

        ////  Movement
        add_points(sp_pose_left_hand_, begin_throw_time, throw_parameter->throw_start_left_arm_);
        add_points(sp_pose_right_hand_, begin_throw_time, throw_parameter->throw_start_right_arm_);
        add_points(sp_pose_trunk_, begin_throw_time, throw_parameter->throw_start_trunk_);
        add_points(sp_pose_left_feet_, begin_throw_time, throw_parameter->start_left_feet_);
        add_points(sp_pose_right_feet_, begin_throw_time, throw_parameter->start_right_feet_);

        /////  Clean Up
        time = begin_throw_time;
    }

    void ThrowCurve::calculate_throw_movement(double & time, std::shared_ptr<ThrowParameter> & throw_parameter){
        /////  Preparation
        //Set up the trajectories for the half cycle (single step)
        double zenith_throw_time =
                time + 1 / (2 * throw_parameter->throw_duration_share_ * throw_parameter->movement_cycle_frequency_);
        double release_throw_time = zenith_throw_time + 1 / (2 * throw_parameter->throw_duration_share_ *
                                                             throw_parameter->movement_cycle_frequency_);

        ////  Movement
        add_points(sp_pose_left_hand_, zenith_throw_time, throw_parameter->throw_zenith_left_arm_);
        add_points(sp_pose_left_hand_, release_throw_time, throw_parameter->throw_release_left_arm_);
        add_points(sp_pose_right_hand_, zenith_throw_time, throw_parameter->throw_zenith_right_arm_);
        add_points(sp_pose_right_hand_, release_throw_time, throw_parameter->throw_release_right_arm_);
        add_points(sp_pose_trunk_, release_throw_time, throw_parameter->throw_release_trunk_);

        /////  Clean Up
        time = release_throw_time;
    }

    void
    ThrowCurve::calculate_throw_conclusion_movement(double & time, std::shared_ptr<ThrowParameter> & throw_parameter){
        /////  Preparation
        //Set up the trajectories for the half cycle (single step)
        double finish_time = time + 1 / (throw_parameter->throw_conclusion_duration_share_ *
                                         throw_parameter->movement_cycle_frequency_);

        ////  Movement
        add_points(sp_pose_left_hand_, finish_time, throw_parameter->start_left_arm_);
        add_points(sp_pose_right_hand_, finish_time, throw_parameter->start_right_arm_);
        add_points(sp_pose_trunk_, finish_time, Struct3dRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

        /////  Clean Up
        time = finish_time;
    }

    void ThrowCurve::add_points(std::shared_ptr<bitbots_splines::PoseHandle> & pose, double const & time,
                                Struct3dRPY const & values){
        pose->x()->add_point(time, values.x_);
        pose->y()->add_point(time, values.y_);
        pose->z()->add_point(time, values.z_);
        pose->roll()->add_point(time, values.roll_);
        pose->pitch()->add_point(time, values.pitch_);
        pose->yaw()->add_point(time, values.yaw_);
    }
} //bitbots_throw