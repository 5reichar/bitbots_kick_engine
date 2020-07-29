#include "throws/throw_curves/throw_curve.h"

namespace bitbots_throw{
    ThrowCurve::ThrowCurve(std::shared_ptr<bitbots_splines::PoseHandle> left_hand
                          ,std::shared_ptr<bitbots_splines::PoseHandle> right_hand
                          ,std::shared_ptr<bitbots_splines::PoseHandle> trunk
                          ,std::shared_ptr<bitbots_splines::PoseHandle> left_feet
                          ,std::shared_ptr<bitbots_splines::PoseHandle> right_feet)
            :sp_pose_left_hand_(std::move(left_hand))
            ,sp_pose_right_hand_(std::move(right_hand))
            ,sp_pose_trunk_(std::move(trunk))
            ,sp_pose_left_feet_(std::move(left_feet))
            ,sp_pose_right_feet_(std::move(right_feet)){
    }

    double ThrowCurve::calculate_trajectories(std::shared_ptr<ThrowParameter> & throw_parameter){
        trajectory_time_ = 0.0;

        //checks
        if(check_requirements(throw_parameter)){
            calculate_pick_up_ball_movement(throw_parameter);
            calculate_throw_preparation_movement(throw_parameter);
            calculate_throw_movement(throw_parameter);
            calculate_throw_conclusion_movement(throw_parameter);
        }
        //TODO: implement ROS Error

        return trajectory_time_;
    }

    bool ThrowCurve::check_requirements(std::shared_ptr<ThrowParameter> & throw_parameter){
        return true;
    }

    tf2::Transform ThrowCurve::get_left_hand_transform(double const & time) const{
        return sp_pose_left_hand_->get_tf_transform(time);
    }

    tf2::Transform ThrowCurve::get_right_hand_transform(double const & time) const{
        return sp_pose_right_hand_->get_tf_transform(time);
    }

    tf2::Transform ThrowCurve::get_trunk_transform(double const & time) const{
        return sp_pose_trunk_->get_tf_transform(time);
    }

    tf2::Transform ThrowCurve::get_left_feet_transform(double const & time) const{
        return sp_pose_left_feet_->get_tf_transform(time);
    }

    tf2::Transform ThrowCurve::get_right_feet_transform(double const & time) const{
        return sp_pose_right_feet_->get_tf_transform(time);
    }

    std::string ThrowCurve::get_debug_string() const{
        std::stringstream points_string;

        points_string << "==== Trunk ====" << std::endl;
        points_string << sp_pose_trunk_->get_debug_string() << std::endl;
        points_string << "==== Left Hand ====" << std::endl;
        points_string << sp_pose_left_hand_->get_debug_string() << std::endl;
        points_string << "==== Right Hand ====" << std::endl;
        points_string << sp_pose_right_hand_->get_debug_string() << std::endl;
        points_string << "==== Left Foot ====" << std::endl;
        points_string << sp_pose_left_feet_->get_debug_string() << std::endl;
        points_string << "==== Right Foot ====" << std::endl;
        points_string << sp_pose_right_feet_->get_debug_string() << std::endl;

        return points_string.str();
    }

    void ThrowCurve::calculate_pick_up_ball_movement(std::shared_ptr<ThrowParameter> & throw_parameter){
        /////  Preparation
        //Set up the trajectories for the half cycle (single step)
        double pick_up_ball_time = trajectory_time_ + 1 / (throw_parameter->pick_up_duration_share_ * throw_parameter->movement_cycle_frequency_);

        ////  Movement
        add_points(sp_pose_left_hand_, trajectory_time_, throw_parameter->start_left_arm_);
        add_points(sp_pose_left_hand_, pick_up_ball_time, throw_parameter->pick_up_left_arm_);
        add_points(sp_pose_right_hand_, trajectory_time_, throw_parameter->start_right_arm_);
        add_points(sp_pose_right_hand_, pick_up_ball_time, throw_parameter->pick_up_right_arm_);
        add_points(sp_pose_trunk_, trajectory_time_, Struct3dRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        add_points(sp_pose_trunk_, pick_up_ball_time, throw_parameter->pick_up_trunk_);
        add_points(sp_pose_left_feet_, trajectory_time_, throw_parameter->start_left_feet_);
        add_points(sp_pose_left_feet_, pick_up_ball_time, Struct3dRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        add_points(sp_pose_right_feet_, trajectory_time_, throw_parameter->start_right_feet_);
        add_points(sp_pose_right_feet_, pick_up_ball_time, Struct3dRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        /////  Clean Up
        trajectory_time_ = pick_up_ball_time;
    }

    void
    ThrowCurve::calculate_throw_preparation_movement(std::shared_ptr<ThrowParameter> & throw_parameter){
        /////  Preparation
        //Set up the trajectories for the half cycle (single step)
        double begin_throw_time = trajectory_time_ + 1 / (throw_parameter->throw_preparation_duration_share_ *
                                              throw_parameter->movement_cycle_frequency_);

        ////  Movement
        add_points(sp_pose_left_hand_, begin_throw_time, throw_parameter->throw_start_left_arm_);
        add_points(sp_pose_right_hand_, begin_throw_time, throw_parameter->throw_start_right_arm_);
        add_points(sp_pose_trunk_, begin_throw_time, throw_parameter->throw_start_trunk_);
        add_points(sp_pose_left_feet_, begin_throw_time, throw_parameter->start_left_feet_);
        add_points(sp_pose_right_feet_, begin_throw_time, throw_parameter->start_right_feet_);

        /////  Clean Up
        trajectory_time_ = begin_throw_time;
    }

    void ThrowCurve::calculate_throw_movement(std::shared_ptr<ThrowParameter> & throw_parameter){
        /////  Preparation
        //Set up the trajectories for the half cycle (single step)
        double zenith_throw_time = trajectory_time_ + 1 / (2 * throw_parameter->throw_duration_share_ * throw_parameter->movement_cycle_frequency_);
        double release_throw_time = zenith_throw_time + 1 / (2 * throw_parameter->throw_duration_share_ *
                                                             throw_parameter->movement_cycle_frequency_);

        ////  Movement
        add_points(sp_pose_left_hand_, zenith_throw_time, throw_parameter->throw_zenith_left_arm_);
        add_points(sp_pose_left_hand_, release_throw_time, throw_parameter->throw_release_left_arm_);
        add_points(sp_pose_right_hand_, zenith_throw_time, throw_parameter->throw_zenith_right_arm_);
        add_points(sp_pose_right_hand_, release_throw_time, throw_parameter->throw_release_right_arm_);
        add_points(sp_pose_trunk_, release_throw_time, throw_parameter->throw_release_trunk_);

        /////  Clean Up
        trajectory_time_ = release_throw_time;
    }

    void
    ThrowCurve::calculate_throw_conclusion_movement(std::shared_ptr<ThrowParameter> & throw_parameter){
        /////  Preparation
        //Set up the trajectories for the half cycle (single step)
        double finish_time = trajectory_time_ + 1 / (throw_parameter->throw_conclusion_duration_share_ *
                                                     throw_parameter->movement_cycle_frequency_);

        ////  Movement
        add_points(sp_pose_left_hand_, finish_time, throw_parameter->start_left_arm_);
        add_points(sp_pose_right_hand_, finish_time, throw_parameter->start_right_arm_);
        add_points(sp_pose_trunk_, finish_time, Struct3dRPY(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

        /////  Clean Up
        trajectory_time_ = finish_time;
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