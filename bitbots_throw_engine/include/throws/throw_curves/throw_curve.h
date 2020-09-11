#ifndef BITBOTS_THROW_THROW_CURVE_H
#define BITBOTS_THROW_THROW_CURVE_H

#include <memory>
#include "ros_interface/throw_visualizer.h"
#include "tf2/LinearMath/Transform.h"
#include "../../bitbots_splines_extension/include/handle/pose_handle.h"
#include "throw_service.h"

namespace bitbots_throw{
	class ThrowCurve{
	public:
		ThrowCurve(std::shared_ptr<bitbots_splines::PoseHandle> left_hand
		          ,std::shared_ptr<bitbots_splines::PoseHandle> right_hand
				  ,std::shared_ptr<bitbots_splines::PoseHandle> trunk
                  ,std::shared_ptr<bitbots_splines::PoseHandle> left_feet
                  ,std::shared_ptr<bitbots_splines::PoseHandle> right_feet);

		/**
		 * Calculates the trajectory for the throw movement.
		 *
		 * @param throw_parameter Container with all values need for the calculate.
		 * @return duration of the movement. Returns 0.0 if the check for the parameter fails.
		 */
		double calculate_trajectories(std::shared_ptr<ThrowParameter> & throw_parameter);
		virtual tf2::Transform get_left_hand_transform(double const & time) const;
		virtual tf2::Transform get_right_hand_transform(double const & time) const;
        virtual tf2::Transform get_trunk_transform(double const & time) const;
        virtual tf2::Transform get_left_feet_transform(double const & time) const;
        virtual tf2::Transform get_right_feet_transform(double const & time) const;

        virtual std::string get_debug_string() const;
        std::vector<double> get_movement_stage() const;

        void visualize_curves(std::shared_ptr<ThrowVisualizer> & sp_visualizer);

	protected:
		virtual bool check_requirements(std::shared_ptr<ThrowParameter> & throw_parameter);

		virtual void calculate_pick_up_ball_movement(std::shared_ptr<ThrowParameter> & throw_parameter);
        virtual void calculate_throw_preparation_movement(std::shared_ptr<ThrowParameter> & throw_parameter);
        virtual void calculate_throw_movement(std::shared_ptr<ThrowParameter> & throw_parameter);
		virtual void calculate_throw_conclusion_movement(std::shared_ptr<ThrowParameter> & throw_parameter);

        virtual void add_points(std::shared_ptr<bitbots_splines::PoseHandle> & pose
                               ,double const & time
                               ,Struct3dRPY const & values);

        double trajectory_time_;
        std::vector<double> movement_stage_;

		std::shared_ptr<bitbots_splines::PoseHandle> sp_pose_left_hand_;
		std::shared_ptr<bitbots_splines::PoseHandle> sp_pose_right_hand_;
        std::shared_ptr<bitbots_splines::PoseHandle> sp_pose_trunk_;
        std::shared_ptr<bitbots_splines::PoseHandle> sp_pose_left_feet_;
        std::shared_ptr<bitbots_splines::PoseHandle> sp_pose_right_feet_;

        std::shared_ptr<ThrowService> sp_service;
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_CURVE_H