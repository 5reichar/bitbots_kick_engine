#include "throws/throw_curves/throw_curve.h"
#include "utility/throw_utilities.h"

namespace bitbots_throw{
    ThrowCurve::ThrowCurve(std::shared_ptr<bitbots_splines::PoseHandle> left_hand
                          ,std::shared_ptr<bitbots_splines::PoseHandle> right_hand
                          ,std::shared_ptr<bitbots_splines::PoseHandle> left_feet
                          ,std::shared_ptr<bitbots_splines::PoseHandle> right_feet)
            :sp_pose_left_hand_(std::move(left_hand))
            ,sp_pose_right_hand_(std::move(right_hand))
            ,sp_pose_left_feet_(std::move(left_feet))
            ,sp_pose_right_feet_(std::move(right_feet))
            ,trajectory_time_(0.0){
    }

    void ThrowCurve::init(std::shared_ptr<ThrowService> service){
        sp_service_ = service;
    }

    double ThrowCurve::calculate_trajectories(){
        trajectory_time_ = 0.0;

        calculate_pick_up_ball_movement();
        calculate_throw_preparation_movement();
        calculate_throw_movement();
        calculate_throw_conclusion_movement();

        return trajectory_time_;
    }

    tf2::Transform ThrowCurve::get_left_hand_transform(double const & time) const{
        return sp_pose_left_hand_->get_tf_transform(time);
    }

    tf2::Transform ThrowCurve::get_right_hand_transform(double const & time) const{
        return sp_pose_right_hand_->get_tf_transform(time);
    }

    tf2::Transform ThrowCurve::get_left_feet_transform(double const & time) const{
        return sp_pose_left_feet_->get_tf_transform(time);
    }

    tf2::Transform ThrowCurve::get_right_feet_transform(double const & time) const{
        return sp_pose_right_feet_->get_tf_transform(time);
    }

    std::string ThrowCurve::get_debug_string() const{
        std::stringstream points_string;

        points_string << "----- Left Hand -----, , Added Points" << std::endl;
        points_string << sp_pose_left_hand_->get_debug_csv() << std::endl;
        points_string << "----- Left Hand -----, , Calculated Points" << std::endl;
        points_string << generate_points_csv(*sp_pose_left_hand_, trajectory_time_) << std::endl;


        points_string << "----- Right Hand -----, , Added Points" << std::endl;
        points_string << sp_pose_right_hand_->get_debug_csv() << std::endl;
        points_string << "----- Right Hand -----, , Calculated Points" << std::endl;
        points_string << generate_points_csv(*sp_pose_right_hand_, trajectory_time_) << std::endl;


        points_string << "----- Left Foot -----, , Added Points" << std::endl;
        points_string << sp_pose_left_feet_->get_debug_csv() << std::endl;
        points_string << "----- Left Foot -----, , Calculated Points" << std::endl;
        points_string << generate_points_csv(*sp_pose_left_feet_, trajectory_time_) << std::endl;


        points_string << "----- Right Foot -----, , Added Points" << std::endl;
        points_string << sp_pose_right_feet_->get_debug_csv() << std::endl;
        points_string << "----- Right Foot -----, , Calculated Points" << std::endl;
        points_string << generate_points_csv(*sp_pose_right_feet_, trajectory_time_) << std::endl;

        return points_string.str();
    }

    std::vector<double> ThrowCurve::get_movement_stage() const{
        return movement_stage_;
    }

    void ThrowCurve::visualize_curves(std::shared_ptr<ThrowVisualizer> & sp_visualizer){
        sp_visualizer->display_left_hand(sp_pose_left_hand_);
        sp_visualizer->display_right_hand(sp_pose_right_hand_);
        sp_visualizer->display_left_foot(sp_pose_left_feet_);
        sp_visualizer->display_right_foot(sp_pose_right_feet_);
    }

    void ThrowCurve::calculate_pick_up_ball_movement(){
        /////  Preparation
        //Set up the trajectories for the half cycle (single step)
        auto movement_time = sp_service_->get_movement_time_pick_up_ball();
        double squat_time = trajectory_time_ + (movement_time / 2);
        double pick_up_ball_time = trajectory_time_ + movement_time;

        ////  Movement
        add_points(sp_pose_left_hand_, trajectory_time_, sp_service_->get_left_arm_start());
        add_points(sp_pose_left_hand_, squat_time, sp_service_->get_left_arm_start());
        add_points(sp_pose_left_hand_, pick_up_ball_time, sp_service_->get_left_arm_pick_up());

        add_points(sp_pose_right_hand_, trajectory_time_, sp_service_->get_right_arm_start());
        add_points(sp_pose_right_hand_, squat_time, sp_service_->get_right_arm_start());
        add_points(sp_pose_right_hand_, pick_up_ball_time, sp_service_->get_right_arm_pick_up());

        add_points(sp_pose_left_feet_, trajectory_time_, sp_service_->get_left_foot_start());
        add_points(sp_pose_left_feet_, squat_time, sp_service_->get_left_foot_squat());
        add_points(sp_pose_left_feet_, pick_up_ball_time, sp_service_->get_left_foot_squat());

        add_points(sp_pose_right_feet_, trajectory_time_, sp_service_->get_right_foot_start());
        add_points(sp_pose_right_feet_, squat_time, sp_service_->get_right_foot_squat());
        add_points(sp_pose_right_feet_, pick_up_ball_time, sp_service_->get_right_foot_squat());
        
        /////  Clean Up
        trajectory_time_ = pick_up_ball_time;
        movement_stage_.push_back(trajectory_time_);
    }

    void
    ThrowCurve::calculate_throw_preparation_movement(){
        /////  Preparation
        //Set up the trajectories for the half cycle (single step)
        auto movement_time = sp_service_->get_movement_time_throw_preparation();
        auto move_ball_at_head_height = trajectory_time_ + (movement_time / 3);
        auto move_ball_over_the_height = trajectory_time_ + (2 * movement_time / 3);
        double begin_throw_time = trajectory_time_ + movement_time;

        ////  Movement
        add_points(sp_pose_left_hand_, move_ball_at_head_height, sp_service_->get_left_arm_ball_at_head_height());
        add_points(sp_pose_left_hand_, move_ball_over_the_height, sp_service_->get_left_arm_throw_zenith());
        add_points(sp_pose_left_hand_, begin_throw_time, sp_service_->get_left_arm_throw_start());

        add_points(sp_pose_right_hand_, move_ball_at_head_height, sp_service_->get_right_arm_ball_at_head_height());
        add_points(sp_pose_right_hand_, move_ball_over_the_height, sp_service_->get_right_arm_throw_zenith());
        add_points(sp_pose_right_hand_, begin_throw_time, sp_service_->get_right_arm_throw_start());

        add_points(sp_pose_left_feet_, begin_throw_time, sp_service_->get_left_foot_start());
        add_points(sp_pose_right_feet_, begin_throw_time, sp_service_->get_right_foot_start());

        /////  Clean Up
        trajectory_time_ = begin_throw_time;
        movement_stage_.push_back(trajectory_time_);
    }

    void ThrowCurve::calculate_throw_movement(){
        /////  Preparation
        //Set up the trajectories for the half cycle (single step)
        auto movement_time = sp_service_->get_movement_time_throw();
        double zenith_throw_time = trajectory_time_ + (movement_time / 2);
        double release_throw_time = trajectory_time_ + movement_time;

        ////  Movement
        add_points(sp_pose_left_hand_, zenith_throw_time, sp_service_->get_left_arm_throw_zenith());
        add_points(sp_pose_left_hand_, release_throw_time, sp_service_->get_left_arm_throw_release());
        add_points(sp_pose_right_hand_, zenith_throw_time, sp_service_->get_right_arm_throw_zenith());
        add_points(sp_pose_right_hand_, release_throw_time, sp_service_->get_right_arm_throw_release());

        /////  Clean Up
        trajectory_time_ = release_throw_time;
        movement_stage_.push_back(trajectory_time_);
    }

    void
    ThrowCurve::calculate_throw_conclusion_movement(){
        /////  Preparation
        //Set up the trajectories for the half cycle (single step)
        auto movement_time = sp_service_->get_movement_time_throw_conclusion();
        double finish_time = trajectory_time_ + movement_time;

        ////  Movement
        add_points(sp_pose_left_hand_, finish_time, sp_service_->get_left_arm_start());
        add_points(sp_pose_right_hand_, finish_time, sp_service_->get_right_arm_start());

        /////  Clean Up
        trajectory_time_ = finish_time;
        movement_stage_.push_back(trajectory_time_);
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