#ifndef BITBOTS_THROW_THROW_CURVE_H
#define BITBOTS_THROW_THROW_CURVE_H

#include <memory>
#include "parameter/throw_parameter.h"
#include "../../bitbots_splines_extension/include/handle/pose_handle.h"

namespace bitbots_throw{
	class ThrowCurve{
	public:
		ThrowCurve(std::shared_ptr<bitbots_splines::PoseHandle> left_hand,
				std::shared_ptr<bitbots_splines::PoseHandle> right_hand,
				std::shared_ptr<bitbots_splines::PoseHandle> trunk);

		bool calculate_trajectories(std::shared_ptr<ThrowParameter> & throw_parameter);
		std::shared_ptr<bitbots_splines::PoseHandle> get_pose_left_hand() const;
		std::shared_ptr<bitbots_splines::PoseHandle> get_pose_right_hand() const;
		std::shared_ptr<bitbots_splines::PoseHandle> get_pose_trunk() const;

	protected:
		virtual bool check_requirements(std::shared_ptr<ThrowParameter> & throw_parameter);

		virtual void calculate_pick_up_ball_movement(double & time, std::shared_ptr<ThrowParameter> & throw_parameter);
		virtual void calculate_throw_movement(double & time, std::shared_ptr<ThrowParameter> & throw_parameter);
		virtual void calculate_throw_conclusion_movement(double & time, std::shared_ptr<ThrowParameter> & throw_parameter);

		std::shared_ptr<bitbots_splines::PoseHandle> sp_pose_left_hand_;
		std::shared_ptr<bitbots_splines::PoseHandle> sp_pose_right_hand_;
		std::shared_ptr<bitbots_splines::PoseHandle> sp_pose_trunk_;
	};
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_CURVE_H