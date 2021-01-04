#include "throws/throw_curves/throw_material.h"
#include "utility/throw_utilities.h"

namespace bitbots_throw{
    ThrowMaterial::ThrowMaterial(std::shared_ptr<bitbots_splines::PoseHandle> left_hand
                           ,std::shared_ptr<bitbots_splines::PoseHandle> right_hand
                           ,std::shared_ptr<bitbots_splines::PoseHandle> left_foot
                           ,std::shared_ptr<bitbots_splines::PoseHandle> right_foot){
        duration_ = 0.0;
        map_poses_[RobotJoints::left_hand] = std::move(left_hand);
        map_poses_[RobotJoints::right_hand] = std::move(right_hand);
        map_poses_[RobotJoints::left_foot] = std::move(left_foot);
        map_poses_[RobotJoints::right_foot] = std::move(right_foot);
    }

    void ThrowMaterial::add_point_to_left_hand(const double & time, const Struct3dRPY & values, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        add_point_to_pose(RobotJoints::left_hand, time, values, velocity, acceleration);
    }

    void ThrowMaterial::add_point_to_right_hand(const double & time, const Struct3dRPY & values, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        add_point_to_pose(RobotJoints::right_hand, time, values, velocity, acceleration);
    }

    void ThrowMaterial::add_point_to_left_foot(const double & time, const Struct3dRPY & values, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        add_point_to_pose(RobotJoints::left_foot, time, values, velocity, acceleration);
    }

    void ThrowMaterial::add_point_to_right_foot(const double & time, const Struct3dRPY & values, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        add_point_to_pose(RobotJoints::right_foot, time, values, velocity, acceleration);
    }

    void ThrowMaterial::add_movement_stage(double & time_stage_start){
        movement_stage_.emplace_back(time_stage_start);
    }

    void ThrowMaterial::add_ik_mode(double const & time, IKMode const & mode){
        map_modes_[time] = mode;
    }

    IKMode ThrowMaterial::get_ik_mode(const double & time){
        IKMode mode = IKMode::arms_and_legs_separated;

        for(auto const it : map_modes_){
            if(it.first > time){
                break;
            }
            mode = it.second;
        }

        return mode;
    }

    std::shared_ptr<bitbots_splines::PoseHandle> ThrowMaterial::get_left_hand() const{
        return map_poses_.at(RobotJoints::left_hand);
    }

    tf2::Transform ThrowMaterial::get_left_hand_transform(double const & time) const{
        return map_poses_.at(RobotJoints::left_hand)->get_tf_transform(time);
    }

    std::shared_ptr<bitbots_splines::PoseHandle> ThrowMaterial::get_right_hand() const{
        return map_poses_.at(RobotJoints::right_hand);
    }

    tf2::Transform ThrowMaterial::get_right_hand_transform(double const & time) const{
        return map_poses_.at(RobotJoints::right_hand)->get_tf_transform(time);
    }

    std::shared_ptr<bitbots_splines::PoseHandle> ThrowMaterial::get_left_foot() const{
        return map_poses_.at(RobotJoints::left_foot);
    }

    tf2::Transform ThrowMaterial::get_left_foot_transform(double const & time) const{
        return map_poses_.at(RobotJoints::left_foot)->get_tf_transform(time);
    }

    std::shared_ptr<bitbots_splines::PoseHandle> ThrowMaterial::get_right_foot() const{
        return map_poses_.at(RobotJoints::right_foot);
    }

    tf2::Transform ThrowMaterial::get_right_foot_transform(double const & time) const{
        return map_poses_.at(RobotJoints::right_foot)->get_tf_transform(time);
    }

    double ThrowMaterial::get_duration() const{
        return duration_;
    }

    std::string ThrowMaterial::get_debug_string() const{
        std::stringstream points_string;

        points_string << "IK Modes, ,,, Movement Duration:," << duration_ << std::endl;
        std::stringstream map_keys;
        std::stringstream map_values;
        for(auto const it : map_modes_){
            map_keys << "," << it.first;
            map_values << "," << it.second;
        }
        points_string << "Time" << map_keys.str() << std::endl;
        points_string << "Mode" << map_values.str() << std::endl;

        points_string << "----- Left Hand -----" << std::endl << "Added Points" << std::endl;
        points_string << map_poses_.at(RobotJoints::left_hand)->get_debug_csv() << std::endl;
        points_string << "----- Left Hand -----" << std::endl << "Calculated Points" << std::endl;
        points_string << generate_points_csv(*map_poses_.at(RobotJoints::left_hand), duration_) << std::endl;


        points_string << "----- Right Hand -----" << std::endl << "Added Points" << std::endl;
        points_string << map_poses_.at(RobotJoints::right_hand)->get_debug_csv() << std::endl;
        points_string << "----- Right Hand -----" << std::endl << "Calculated Points" << std::endl;
        points_string << generate_points_csv(*map_poses_.at(RobotJoints::right_hand), duration_) << std::endl;


        points_string << "----- Left Foot -----" << std::endl << "Added Points" << std::endl;
        points_string << map_poses_.at(RobotJoints::left_foot)->get_debug_csv() << std::endl;
        points_string << "----- Left Foot -----" << std::endl << "Calculated Points" << std::endl;
        points_string << generate_points_csv(*map_poses_.at(RobotJoints::left_foot), duration_) << std::endl;


        points_string << "----- Right Foot -----" << std::endl << "Added Points" << std::endl;
        points_string << map_poses_.at(RobotJoints::right_foot)->get_debug_csv() << std::endl;
        points_string << "----- Right Foot -----" << std::endl << "Calculated Points" << std::endl;
        points_string << generate_points_csv(*map_poses_.at(RobotJoints::right_foot), duration_) << std::endl;

        return points_string.str();
    }

    std::vector<double> ThrowMaterial::get_movement_stage() const{
        return movement_stage_;
    }

    void ThrowMaterial::visualize_curves(std::shared_ptr<ThrowVisualizer> & sp_visualizer){
        sp_visualizer->display_left_hand(map_poses_[RobotJoints::left_hand]);
        sp_visualizer->display_right_hand(map_poses_[RobotJoints::right_hand]);
        sp_visualizer->display_left_foot(map_poses_[RobotJoints::left_foot]);
        sp_visualizer->display_right_foot(map_poses_[RobotJoints::right_foot]);
    }

    void ThrowMaterial::add_point_to_pose(RobotJoints joint, double const & time, Struct3dRPY const & values, Struct3dRPY const & velocity, Struct3dRPY const & acceleration){
        add_point(map_poses_[joint], time, values, velocity, acceleration);
        if (duration_ < time){
            duration_ = time;
        }
    }
}