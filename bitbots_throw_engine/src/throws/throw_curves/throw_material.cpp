#include "throws/throw_curves/throw_material.h"
#include "utility/throw_utilities.h"

namespace bitbots_throw{
    ThrowMaterial::ThrowMaterial(std::shared_ptr<bitbots_splines::PoseHandle> left_hand
                           ,std::shared_ptr<bitbots_splines::PoseHandle> right_hand
                           ,std::shared_ptr<bitbots_splines::PoseHandle> left_foot
                           ,std::shared_ptr<bitbots_splines::PoseHandle> right_foot)
            : sp_pose_left_hand_(std::move(left_hand))
            , sp_pose_right_hand_(std::move(right_hand))
            , sp_pose_left_foot_(std::move(left_foot))
            , sp_pose_right_foot_(std::move(right_foot)){
        duration_ = 0.0;
    }

    void ThrowMaterial::add_point_to_left_hand(const double & time, const Struct3dRPY & values, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        add_point_to_pose(sp_pose_left_hand_, time, values, velocity, acceleration);
    }

    void ThrowMaterial::add_point_to_right_hand(const double & time, const Struct3dRPY & values, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        add_point_to_pose(sp_pose_right_hand_, time, values, velocity, acceleration);
    }

    void ThrowMaterial::add_point_to_left_foot(const double & time, const Struct3dRPY & values, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        add_point_to_pose(sp_pose_left_foot_, time, values, velocity, acceleration);
    }

    void ThrowMaterial::add_point_to_right_foot(const double & time, const Struct3dRPY & values, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        add_point_to_pose(sp_pose_right_foot_, time, values, velocity, acceleration);
    }

    void ThrowMaterial::add_movement_stage(double & time_stage_start){
        movement_stage_.emplace_back(time_stage_start);
    }

    std::shared_ptr<bitbots_splines::PoseHandle> ThrowMaterial::get_left_hand() const{
        return sp_pose_left_hand_;
    }

    tf2::Transform ThrowMaterial::get_left_hand_transform(double const & time) const{
        return sp_pose_left_hand_->get_tf_transform(time);
    }

    std::shared_ptr<bitbots_splines::PoseHandle> ThrowMaterial::get_right_hand() const{
        return sp_pose_right_hand_;
    }

    tf2::Transform ThrowMaterial::get_right_hand_transform(double const & time) const{
        return sp_pose_right_hand_->get_tf_transform(time);
    }

    std::shared_ptr<bitbots_splines::PoseHandle> ThrowMaterial::get_left_foot() const{
        return sp_pose_left_foot_;
    }

    tf2::Transform ThrowMaterial::get_left_foot_transform(double const & time) const{
        return sp_pose_left_foot_->get_tf_transform(time);
    }

    std::shared_ptr<bitbots_splines::PoseHandle> ThrowMaterial::get_right_foot() const{
        return sp_pose_right_foot_;
    }

    tf2::Transform ThrowMaterial::get_right_foot_transform(double const & time) const{
        return sp_pose_right_foot_->get_tf_transform(time);
    }

    double ThrowMaterial::get_duration() const{
        return duration_;
    }

    std::string ThrowMaterial::get_debug_string() const{
        std::stringstream points_string;

        points_string << "----- Left Hand -----" << std::endl << "Added Points" << std::endl;
        points_string << sp_pose_left_hand_->get_debug_csv() << std::endl;
        points_string << "----- Left Hand -----" << std::endl << "Calculated Points" << std::endl;
        points_string << generate_points_csv(*sp_pose_left_hand_, duration_) << std::endl;


        points_string << "----- Right Hand -----" << std::endl << "Added Points" << std::endl;
        points_string << sp_pose_right_hand_->get_debug_csv() << std::endl;
        points_string << "----- Right Hand -----" << std::endl << "Calculated Points" << std::endl;
        points_string << generate_points_csv(*sp_pose_right_hand_, duration_) << std::endl;


        points_string << "----- Left Foot -----" << std::endl << "Added Points" << std::endl;
        points_string << sp_pose_left_foot_->get_debug_csv() << std::endl;
        points_string << "----- Left Foot -----" << std::endl << "Calculated Points" << std::endl;
        points_string << generate_points_csv(*sp_pose_left_foot_, duration_) << std::endl;


        points_string << "----- Right Foot -----" << std::endl << "Added Points" << std::endl;
        points_string << sp_pose_right_foot_->get_debug_csv() << std::endl;
        points_string << "----- Right Foot -----" << std::endl << "Calculated Points" << std::endl;
        points_string << generate_points_csv(*sp_pose_right_foot_, duration_) << std::endl;

        return points_string.str();
    }

    std::vector<double> ThrowMaterial::get_movement_stage() const{
        return movement_stage_;
    }

    void ThrowMaterial::visualize_curves(std::shared_ptr<ThrowVisualizer> & sp_visualizer){
        sp_visualizer->display_left_hand(sp_pose_left_hand_);
        sp_visualizer->display_right_hand(sp_pose_right_hand_);
        sp_visualizer->display_left_foot(sp_pose_left_foot_);
        sp_visualizer->display_right_foot(sp_pose_right_foot_);
    }

    void ThrowMaterial::add_point_to_pose(std::shared_ptr<bitbots_splines::PoseHandle> & pose, double const & time, Struct3dRPY const & values, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        add_point(pose, time, values, velocity, acceleration);
        if (duration_ < time){
            duration_ = time;
        }
    }
}