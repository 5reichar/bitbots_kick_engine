#ifndef BITBOTS_THROW_THROW_MATERIAL_H
#define BITBOTS_THROW_THROW_MATERIAL_H

#include "utility/throw_struct.h"
#include "tf2/LinearMath/Transform.h"
#include "ros_interface/throw_visualizer.h"
#include "../../bitbots_splines_extension/include/handle/pose_handle.h"

namespace bitbots_throw{
    class ThrowMaterial{
    public:
        ThrowMaterial(std::shared_ptr<bitbots_splines::PoseHandle> left_hand
            ,std::shared_ptr<bitbots_splines::PoseHandle> right_hand
            ,std::shared_ptr<bitbots_splines::PoseHandle> left_foot
            ,std::shared_ptr<bitbots_splines::PoseHandle> right_foot);

        void add_point_to_left_hand(double const & time, Struct3dRPY const & values, Struct3dRPY const & velocity, Struct3dRPY const & acceleration);
        void add_point_to_right_hand(double const & time, Struct3dRPY const & values, Struct3dRPY const & velocity, Struct3dRPY const & acceleration);
        void add_point_to_left_foot(double const & time, Struct3dRPY const & values, Struct3dRPY const & velocity, Struct3dRPY const & acceleration);
        void add_point_to_right_foot(double const & time, Struct3dRPY const & values, Struct3dRPY const & velocity, Struct3dRPY const & acceleration);

        void add_movement_stage(double & time_stage_start);

        std::shared_ptr<bitbots_splines::PoseHandle> get_left_hand() const;
        virtual tf2::Transform get_left_hand_transform(double const & time) const;
        std::shared_ptr<bitbots_splines::PoseHandle> get_right_hand() const;
        virtual tf2::Transform get_right_hand_transform(double const & time) const;
        std::shared_ptr<bitbots_splines::PoseHandle> get_left_foot() const;
        virtual tf2::Transform get_left_foot_transform(double const & time) const;
        std::shared_ptr<bitbots_splines::PoseHandle> get_right_foot() const;
        virtual tf2::Transform get_right_foot_transform(double const & time) const;

        virtual double get_duration() const;
        virtual std::string get_debug_string() const;
        virtual std::vector<double> get_movement_stage() const;

        void visualize_curves(std::shared_ptr<ThrowVisualizer> & sp_visualizer);

    private:
        void add_point_to_pose(std::shared_ptr<bitbots_splines::PoseHandle> & pose, double const & time, Struct3dRPY const & values, Struct3dRPY const & velocity, Struct3dRPY const & acceleration);

        double duration_;
        std::vector<double> movement_stage_;

        std::shared_ptr<bitbots_splines::PoseHandle> sp_pose_left_hand_;
        std::shared_ptr<bitbots_splines::PoseHandle> sp_pose_right_hand_;
        std::shared_ptr<bitbots_splines::PoseHandle> sp_pose_left_foot_;
        std::shared_ptr<bitbots_splines::PoseHandle> sp_pose_right_foot_;

    };
}
#endif //BITBOTS_THROW_THROW_MATERIAL_H
